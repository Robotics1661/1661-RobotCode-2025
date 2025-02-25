package frc.robot.subsystems;

import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.util.LimitedTalonFX;

@SuppressWarnings("unused")
public class ManipulatorSubsystem extends CustomSubsystemBase {
    private final LimitedTalonFX m_wrist = new LimitedTalonFX(new TalonFX(Constants.MANIPULATOR_WRIST), this::wristForwardStop, this::wristBackwardStop);
    private final TalonFX m_intake = new TalonFX(Constants.MANIPULATOR_INTAKE);
    private final CANcoder m_encoder = new CANcoder(Constants.MANIPULATOR_ENCODER);

    private double m_intake_rate = 0.0;
    /** Whether the coral/algae has reached the back of the intake. Detected by current spikes */
    private boolean m_intake_reached = false;

    public ManipulatorSubsystem() {
        setSubsystem("manipulator");

        ntPutData(builder -> {
            builder.addDoubleProperty("wrist/encoder rotations", () -> m_encoder.getPosition().getValueAsDouble(), null);
            builder.addBooleanProperty("wrist/at max", this::wristForwardStop, null);
            builder.addBooleanProperty("wrist/at min", this::wristBackwardStop, null);

            builder.addDoubleProperty("intake/current (Amps)", () -> m_intake.getTorqueCurrent().getValueAsDouble(), null);
            builder.addBooleanProperty("intake/spiking", () -> isIntakeCurrentSpiking(), null);
            builder.addBooleanProperty("intake/back reached", () -> m_intake_reached, null);
        });

        var wristConf = m_wrist.m_wrapped.getConfigurator();
        wristConf.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        );

        var intakeConf = m_intake.getConfigurator();
        intakeConf.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // positive means intake
            .withNeutralMode(NeutralModeValue.Brake)
        );
    }

    @Override
    public void periodic() {
        m_wrist.periodic();

        if (!m_intake_reached && isIntakeCurrentSpiking()) {
            m_intake_reached = true;
            m_intake.setControl(STATIC_BRAKE);
        }
    }

    // motion methods

    public void runIntake(double rate) {
        rate = clamp(rate, -1, 1);

        if (Math.abs(rate - m_intake_rate) > 0.01) {
            m_intake_reached = false;
        }

        m_intake_rate = rate;

        if (!m_intake_reached) {
            if (Math.abs(rate) > 0.01) {
                m_intake.setControl(getPooledDutyCycleOut(m_intake, rate));
            } else {
                m_intake.setControl(STATIC_BRAKE);
            }
        }
    }

    public void moveWristTo(WristSetPoint setPoint) {
        m_wrist.setControl(getPooledPositionVoltage(m_wrist, setPoint.rotationValue));
    }

    public void disableStop() {
        m_wrist.setControl(STATIC_BRAKE);
        m_intake.setControl(COAST_OUT);
    }

    // Limits

    protected boolean wristForwardStop() {
        return m_encoder.getPosition().getValue().baseUnitMagnitude() >= Constants.MANIPULATOR_WRIST_MAX.baseUnitMagnitude();
    }

    protected boolean wristBackwardStop() {
        return m_encoder.getPosition().getValue().baseUnitMagnitude() <= Constants.MANIPULATOR_WRIST_MIN.baseUnitMagnitude();
    }

    protected boolean isIntakeCurrentSpiking() {
        if (m_intake_rate < 0.0) { // irrelevant when outputting
            return false;
        }
        return m_intake.getTorqueCurrent().getValueAsDouble() > 30.0;
    }

    // Set Points
    // 0.387 - 0.622 rotations
    public enum WristSetPoint {
        INTAKE(0.56),
        TEST_LOW(0.4)
        ;
        public final Angle rotationValue;

        WristSetPoint(double rotationValue) {
            this.rotationValue = Units.Rotation.of(rotationValue);
        }
    }
}
