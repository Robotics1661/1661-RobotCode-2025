package frc.robot.subsystems;

import static frc.robot.util.MathUtil.clamp;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.LimitedTalonFX;

public class WristSubsystem extends CustomSubsystemBase {
    private final LimitedTalonFX m_wrist = new LimitedTalonFX(new TalonFX(Constants.MANIPULATOR_WRIST), this::wristForwardStop, this::wristBackwardStop);
    private final CANcoder m_encoder = new CANcoder(Constants.MANIPULATOR_ENCODER);

    private final double kG = 0.175;
    private final double kP = 225.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double mmCruiseVel = 4.0;
    private final double mmAcceleration = 6.0;

    private Angle activeBrakeTarget = null;

    private double calibPos = (Constants.MANIPULATOR_WRIST_MAX.in(Units.Rotations) + Constants.MANIPULATOR_WRIST_MIN.in(Units.Rotations)) / 2.0;

    private Timer m_copy_pos_timer = new Timer();

    public WristSubsystem() {
        setSubsystem("wrist");

        ntPutData(builder -> {
            /*builder.addDoubleProperty("PID/kG", () -> kG, pidSetter(v -> kG = v));
            builder.addDoubleProperty("PID/kP", () -> kP, pidSetter(v -> kP = v));
            builder.addDoubleProperty("PID/kI", () -> kI, pidSetter(v -> kI = v));
            builder.addDoubleProperty("PID/kD", () -> kD, pidSetter(v -> kD = v));
            builder.addDoubleProperty("PID/mmCruiseVel", () -> mmCruiseVel, pidSetter(v -> mmCruiseVel = v));
            builder.addDoubleProperty("PID/mmAcceleration", () -> mmAcceleration, pidSetter(v -> mmAcceleration = v));*/

            builder.addDoubleProperty("Calib Pos", () -> calibPos, null);

            builder.addDoubleProperty("encoder rotations", () -> m_encoder.getAbsolutePosition().getValueAsDouble(), null);
            builder.addDoubleProperty("motor encoder rotations", () -> m_wrist.m_wrapped.getPosition().getValueAsDouble(), null);
            builder.addBooleanProperty("at max", this::wristForwardStop, null);
            builder.addBooleanProperty("at min", this::wristBackwardStop, null);

            double[] lastError = new double[] {0.0};
            builder.addDoubleProperty("PID/Error", () -> {
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

        m_copy_pos_timer.reset();
        m_copy_pos_timer.start();

        configurePID();

        ntPutString("Control", "StaticBrake");
    }

    @SuppressWarnings("unused")
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

        m_wrist.periodic();
    }

    // motion methods

    public void calibUp() {
        calibPos += 0.01;
        calibPos = clamp(
            calibPos,
            Constants.MANIPULATOR_WRIST_MIN.in(Units.Rotations),
            Constants.MANIPULATOR_WRIST_MAX.in(Units.Rotations)
        );
    }

    public void calibDown() {
        calibPos += 0.01;
        calibPos = clamp(
            calibPos,
            Constants.MANIPULATOR_WRIST_MIN.in(Units.Rotations),
            Constants.MANIPULATOR_WRIST_MAX.in(Units.Rotations)
        );
    }

    /**
     * Move wrist to setpoint.
     * @param setPoint point to move to
     * @return whether we have reached the target
     */
    public boolean moveWristTo(WristSetPoint setPoint) {
        if (m_copy_pos_timer != null) {
            return false;
        }
        ntPutString("Control", "MotionMagicVoltage("+setPoint+")");
        return moveWristToInternal(setPoint.rotationValue);
    }

    /**
     * Move wrist to set angle. {@link #moveWristTo(WristSetPoint)} is prefered
     * @param angle angle to move to
     * @return whether we have reached the target
     */
    public boolean moveWristTo(Angle angle) {
        if (m_copy_pos_timer != null) {
            return false;
        }
        ntPutString("Control", "MotionMagicVoltage("+angle+")");
        return moveWristToInternal(angle);
    }

    private static final double epsilonRot = 0.01;

    private boolean moveWristToInternal(Angle angle) {
        if (m_copy_pos_timer != null) {
            return false;
        }

        angle = clamp(angle, Constants.MANIPULATOR_WRIST_MIN, Constants.MANIPULATOR_WRIST_MAX);

        m_wrist.setControl(getPooledMotionMagicVoltage(null, angle));
        activeBrakeTarget = null;

        return m_wrist.m_wrapped.getPosition().getValue().minus(angle).abs(Units.Rotations) < epsilonRot;
        //m_wrist.setControl(getPooledPositionVoltage(null, setPoint.rotationValue));
    }

    public boolean atTarget() {
        if (m_wrist.m_wrapped.getAppliedControl() instanceof MotionMagicVoltage mmv) {
            return m_wrist.m_wrapped.getPosition().getValue()
                .minus(mmv.getPositionMeasure())
                .abs(Units.Rotations) < epsilonRot;
        }
        return false;
    }

    public void moveToCalib() {
        if (m_copy_pos_timer != null) {
            return;
        }
        calibPos = clamp(calibPos, Constants.MANIPULATOR_WRIST_MIN.in(Units.Rotations), Constants.MANIPULATOR_WRIST_MAX.in(Units.Rotations));
        ntPutString("Control", "MoveTo("+calibPos+")");

        m_wrist.setControl(getPooledMotionMagicVoltage(null, Units.Rotations.of(calibPos)));
        activeBrakeTarget = null;
    }

    public void smartMoveRate(double rate) {
        rate = clamp(rate, -1., 1.);
        if (Math.abs(rate) < 0.001) {
            if (m_copy_pos_timer == null) {
                activeBrake();
            } else {
                ntPutString("Control", "StaticBrake");
                m_wrist.setControl(STATIC_BRAKE);
                activeBrakeTarget = null;
            }
        } else {
            ntPutString("Control", "DutyCycleOut("+rate+")");
            m_wrist.setControl(getPooledDutyCycleOut(null, rate));
            activeBrakeTarget = null;
        }
    }

    private void activeBrake() {
        if (activeBrakeTarget == null) {
            activeBrakeTarget = clamp(m_wrist.m_wrapped.getPosition().getValue(), Constants.MANIPULATOR_WRIST_MIN, Constants.MANIPULATOR_WRIST_MAX);
        }
        ntPutString("Control", "ActiveBrake("+activeBrakeTarget+")");
        m_wrist.setControl(getPooledMotionMagicVoltage(null, activeBrakeTarget));
    }

    public void moveRate(double rate) {
        rate = clamp(rate, -1.0, 1.0);
        ntPutString("Control", "DutyCycleOut("+rate+")");

        m_wrist.setControl(getPooledDutyCycleOut(null, rate));
        activeBrakeTarget = null;
    }

    public void stop() {
        ntPutString("Control", "StaticBrake");
        m_wrist.setControl(STATIC_BRAKE);
        activeBrakeTarget = null;
    }

    public void disableStop() {
        ntPutString("Control", "StaticBrake");
        m_wrist.setControl(STATIC_BRAKE);
        activeBrakeTarget = null;
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

    public boolean withinSafeRange() {
        return !wristForwardStop() && !wristBackwardStop();
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
