package frc.robot.subsystems;

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

    public ManipulatorSubsystem() {
        setSubsystem("manipulator");

        ntPutData(builder -> {
            builder.addDoubleProperty("encoder rotations", () -> m_encoder.getPosition().getValueAsDouble(), null);
            builder.addBooleanProperty("at max", this::wristForwardStop, null);
            builder.addBooleanProperty("at min", this::wristBackwardStop, null);
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
    }

    // motion methods

    public void moveWristTo(WristSetPoint setPoint) {
        m_wrist.setControl(getPooledPositionVoltage(null, setPoint.rotationValue));
    }

    public void disableStop() {
        m_wrist.setControl(STATIC_BRAKE);
        m_intake.setControl(COAST_OUT);
    }

    // Limits

    protected boolean wristForwardStop() {
        return m_encoder.getAbsolutePosition().getValue().baseUnitMagnitude() >= Constants.MANIPULATOR_WRIST_MAX.baseUnitMagnitude();
    }

    protected boolean wristBackwardStop() {
        return m_encoder.getAbsolutePosition().getValue().baseUnitMagnitude() <= Constants.MANIPULATOR_WRIST_MIN.baseUnitMagnitude();
    }

    // Set Points
    // 0.387 - 0.622 rotations
    public enum WristSetPoint {
        INTAKE(0.56),
        TEST_LOW(0.4)
        ;
        private final Angle rotationValue;

        WristSetPoint(double rotationValue) {
            this.rotationValue = Units.Rotation.of(rotationValue);
        }
    }
}
