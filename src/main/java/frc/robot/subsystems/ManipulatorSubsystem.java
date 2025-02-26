package frc.robot.subsystems;

import static frc.robot.util.MathUtil.clamp;
import static frc.robot.util.MathUtil.wrap_0_1;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.LimitedTalonFX;

@SuppressWarnings("unused")
public class ManipulatorSubsystem extends CustomSubsystemBase {
    private final LimitedTalonFX m_wrist = new LimitedTalonFX(new TalonFX(Constants.MANIPULATOR_WRIST), this::wristForwardStop, this::wristBackwardStop);
    private final TalonFX m_intake = new TalonFX(Constants.MANIPULATOR_INTAKE);
    private final CANcoder m_encoder = new CANcoder(Constants.MANIPULATOR_ENCODER);

    // Period is 0.02 seconds - this is the standard FRC main loop period
    private final LinearFilter m_intake_current_filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private double m_intake_rate = 0.0;
    /** Whether the coral/algae has reached the back of the intake. Detected by current spikes */
    private boolean m_intake_reached = false;

    private double kG = 0.175;
    private double kP = 225.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double mmCruiseVel = 4.0;
    private double mmAcceleration = 6.0;

    private Timer m_copy_pos_timer = new Timer();

    public ManipulatorSubsystem() {
        setSubsystem("manipulator");

        ntPutData(builder -> {
            /*builder.addDoubleProperty("wrist/PID/kG", () -> kG, pidSetter(v -> kG = v));
            builder.addDoubleProperty("wrist/PID/kP", () -> kP, pidSetter(v -> kP = v));
            builder.addDoubleProperty("wrist/PID/kI", () -> kI, pidSetter(v -> kI = v));
            builder.addDoubleProperty("wrist/PID/kD", () -> kD, pidSetter(v -> kD = v));
            builder.addDoubleProperty("wrist/PID/mmCruiseVel", () -> mmCruiseVel, pidSetter(v -> mmCruiseVel = v));
            builder.addDoubleProperty("wrist/PID/mmAcceleration", () -> mmAcceleration, pidSetter(v -> mmAcceleration = v));*/

            builder.addDoubleProperty("wrist/encoder rotations", () -> m_encoder.getAbsolutePosition().getValueAsDouble(), null);
            builder.addDoubleProperty("wrist/motor encoder rotations", () -> m_wrist.m_wrapped.getPosition().getValueAsDouble(), null);
            builder.addBooleanProperty("wrist/at max", this::wristForwardStop, null);
            builder.addBooleanProperty("wrist/at min", this::wristBackwardStop, null);

            builder.addDoubleProperty("intake/current (Amps)", () -> m_intake.getTorqueCurrent().getValueAsDouble(), null);
            builder.addDoubleProperty("intake/filt current (Amps)", () -> m_intake_current_filter.lastValue(), null);
            builder.addBooleanProperty("intake/spiking", () -> isIntakeCurrentSpiking(), null);
            builder.addBooleanProperty("intake/back reached", () -> m_intake_reached, null);

            double[] lastError = new double[] {0.0};
            builder.addDoubleProperty("wrist/PID/Error", () -> {
                if (m_wrist.m_wrapped.getAppliedControl() instanceof PositionVoltage pv) {
                    lastError[0] = m_wrist.m_wrapped.getPosition().getValueAsDouble() - pv.Position;
                    return lastError[0];
                } else if (m_wrist.m_wrapped.getAppliedControl() instanceof MotionMagicVoltage mmv) {
                    lastError[0] = m_wrist.m_wrapped.getPosition().getValueAsDouble() - mmv.Position;
                    return lastError[0];
                } else {
                    return lastError[0];
                }
            }, null);
        });

        final double wristFactor = 1.0 / 15.0 * (28.0 / 76.0);

        var encoderConf = m_encoder.getConfigurator();
        encoderConf.apply(new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(1.0))
        );

        var wristConf = m_wrist.m_wrapped.getConfigurator();
        wristConf.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        );
        wristConf.apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(1. / wristFactor)
        );

        var intakeConf = m_intake.getConfigurator();
        intakeConf.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // positive means intake
            .withNeutralMode(NeutralModeValue.Brake)
        );

        m_copy_pos_timer.reset();
        m_copy_pos_timer.start();

        configurePID();

        ntPutString("wrist/Control", "StaticBrake");
    }

    private DoubleConsumer pidSetter(DoubleConsumer setter) {
        return v -> {
            setter.accept(v);
            configurePID();
        };
    }

    private void configurePID() {
        var motorConfig = m_wrist.m_wrapped.getConfigurator();
        motorConfig.apply(new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(kG)
            .withKS(0.0)
            .withKA(0.0)
            .withKV(0.0)
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
        );

        motorConfig.apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Units.RotationsPerSecond.of(mmCruiseVel))
            .withMotionMagicAcceleration(Units.RotationsPerSecondPerSecond.of(mmAcceleration))
        );
    }

    @Override
    public void periodic() {
        if (m_copy_pos_timer != null && m_copy_pos_timer.hasElapsed(0.25)) {
            m_wrist.m_wrapped.setPosition(m_encoder.getAbsolutePosition().getValue());
            m_copy_pos_timer = null;
        }

        m_intake_current_filter.calculate(m_intake.getTorqueCurrent().getValueAsDouble());
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
                m_intake.setControl(getPooledVoltageOut(m_intake, Units.Volt.of(rate * 12.0)));
            } else {
                m_intake.setControl(STATIC_BRAKE);
            }
        }
    }

    public void moveWristTo(WristSetPoint setPoint) {
        ntPutString("wrist/Control", "MotionMagicVoltage("+setPoint+")");

        m_wrist.setControl(getPooledMotionMagicVoltage(m_wrist, setPoint.rotationValue));
        //m_wrist.setControl(getPooledPositionVoltage(m_wrist, setPoint.rotationValue));
    }

    public void moveWrist(double rate) {
        rate = clamp(rate, -1.0, 1.0);
        ntPutString("wrist/Control", "DutyCycleOut("+rate+")");

        m_wrist.setControl(getPooledDutyCycleOut(m_wrist, rate));
    }

    public void stopWrist() {
        ntPutString("wrist/Control", "StaticBreak");
        m_wrist.setControl(STATIC_BRAKE);
    }

    public void disableStop() {
        ntPutString("wrist/Control", "StaticBreak");
        m_wrist.setControl(STATIC_BRAKE);
        m_intake.setControl(COAST_OUT);
    }

    // Limits

    protected boolean wristForwardStop() {
        double encoderPos = Math.max(
            m_encoder.getAbsolutePosition().getValue().baseUnitMagnitude(),
            m_wrist.m_wrapped.getPosition().getValue().baseUnitMagnitude()
        );
        return encoderPos >= Constants.MANIPULATOR_WRIST_MAX.baseUnitMagnitude();
    }

    protected boolean wristBackwardStop() {
        double encoderPos = Math.min(
            m_encoder.getAbsolutePosition().getValue().baseUnitMagnitude(),
            m_wrist.m_wrapped.getPosition().getValue().baseUnitMagnitude()
        );
        return encoderPos <= Constants.MANIPULATOR_WRIST_MIN.baseUnitMagnitude();
    }

    protected boolean isIntakeCurrentSpiking() {
        if (m_intake_rate <= 0.0) { // irrelevant when outputting
            return false;
        }
        return m_intake_current_filter.lastValue() > 60.0;
    }

    // Set Points
    // 0.387 - 0.622 rotations
    public enum WristSetPoint {
        INTAKE(0.56),
        TEST_LOW(0.5)
        ;
        public final Angle rotationValue;

        WristSetPoint(double rotationValue) {
            this.rotationValue = Units.Rotation.of(rotationValue);
        }

        public String toString() {
            return name() + "(" + rotationValue + ")";
        }
    }
}
