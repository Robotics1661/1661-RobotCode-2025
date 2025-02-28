package frc.robot.subsystems;

import static frc.robot.util.MathUtil.clamp;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LimitedTalonFX;

public class ElevatorSubsystem extends CustomSubsystemBase {
    private final LimitedTalonFX m_motor = new LimitedTalonFX(new TalonFX(Constants.ELEVATOR_MOTOR), this::atTop, this::atBottom);
    // Switches return FALSE when pressed.
    private final DigitalInput m_topLimitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_TOP);
    private final DigitalInput m_warnLimitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_WARN);
    private final DigitalInput m_bottomLimitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_BOTTOM);

    /** Whether we've zeroed to the bottom yet */
    private Timer m_callibrationSettleTimer = null;
    private boolean m_callibrated = false;

    private Angle activeBrakeTarget = null;

    private final double kP = 8.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double mmCruiseVel = 80.0;
    private final double mmAcceleration = 120.0;

    private double calibPos = 40.0;

    public ElevatorSubsystem() {
        setSubsystem("elevator");

        // configure the motor to behave itself properly
        var motorConfig = m_motor.m_wrapped.getConfigurator();
        motorConfig.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // we want positive to correspond to up
            .withNeutralMode(NeutralModeValue.Brake) // don't just slide down when nothing is being output
        );
        configurePID();

        m_motor.setControl(COAST_OUT);

        ntPutData(builder -> {
            /*builder.addDoubleProperty("PID/kP", () -> kP, pidSetter(v -> kP = v));
            builder.addDoubleProperty("PID/kI", () -> kI, pidSetter(v -> kI = v));
            builder.addDoubleProperty("PID/kD", () -> kD, pidSetter(v -> kD = v));
            builder.addDoubleProperty("PID/mmCruiseVel", () -> mmCruiseVel, pidSetter(v -> mmCruiseVel = v));
            builder.addDoubleProperty("PID/mmAcceleration", () -> mmAcceleration, pidSetter(v -> mmAcceleration = v));*/

            builder.addDoubleProperty("Calib Pos", () -> calibPos, null);

            builder.addBooleanProperty("At Top", this::atTop, null);
            builder.addBooleanProperty("In Bottom Warn", this::inWarn, null);
            builder.addBooleanProperty("At Bottom", this::atBottom, null);
            builder.addStringProperty("Position", () -> m_motor.m_wrapped.getPosition().toString(), null);
            builder.addBooleanProperty("Callibrated", () -> m_callibrated, null);

            double[] lastError = new double[] {0.0};
            builder.addDoubleProperty("PID/Error", () -> {
                if (m_motor.m_wrapped.getAppliedControl() instanceof PositionVoltage pv) {
                    lastError[0] = m_motor.m_wrapped.getPosition().getValueAsDouble() - pv.Position;
                    return lastError[0];
                } else if (m_motor.m_wrapped.getAppliedControl() instanceof MotionMagicVoltage mmv) {
                    lastError[0] = m_motor.m_wrapped.getPosition().getValueAsDouble() - mmv.Position;
                    return lastError[0];
                } else {
                    return lastError[0];
                }
            }, null);
        });

        ntPutString("Control", "None");
        ntPutNumber("smart move limit", 1.0);
    }

    /*private DoubleConsumer pidSetter(DoubleConsumer setter) {
        return v -> {
            setter.accept(v);
            configurePID();
        };
    }*/

    private void configurePID() {
        var motorConfig = m_motor.m_wrapped.getConfigurator();
        motorConfig.apply(new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0.41198) // tuned with SysId
            .withKS(0.01402)
            .withKA(0.013195)
            .withKV(0.11844)
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
        m_motor.periodic();
    }

    // Callibration

    public boolean isCallibrated() {
        return m_callibrated;
    }

    public void startCallibration() {
        m_callibrated = false;
        log("starting callibration");
        activeBrakeTarget = null;
    }

    /**
     * Try to complete callibration process
     * @return whether the elevator has in fact reached the bottom
     */
    public boolean completeCallibration() {
        if (!(atBottom() || Utils.isSimulation())) {
            m_callibrationSettleTimer = null;
            return false;
        }

        if (m_callibrationSettleTimer == null) {
            m_callibrationSettleTimer = new Timer();
            m_callibrationSettleTimer.start();
            log("callibration settling");
        }

        if (!m_callibrationSettleTimer.hasElapsed(1)) {
            return false;
        }

        stop();
        m_motor.m_wrapped.setPosition(0);
        m_callibrated = true;
        m_callibrationSettleTimer = null;

        log("callibration complete");

        return true;
    }

    // High-level motion methods
    
    public void calibUp() {
        calibPos += 0.5;
    }

    public void calibDown() {
        calibPos -= 0.5;
    }

    public enum ElevatorSetPoint {
        BASE(1.0),
        TESTING(40.0)
        ;
        public final Angle rotationValue;

        ElevatorSetPoint(double rotationValue) {
            this.rotationValue = Units.Rotation.of(rotationValue);
        }

        public String toString() {
            return name() + "(" + rotationValue + ")";
        }
    }

    /**
     * Move to setpoint.
     * @param setPoint point to move to
     * @return whether we have reached the target
     */
    public boolean moveTo(ElevatorSetPoint setPoint) {
        if (!m_callibrated) {
            return false;
        }
        ntPutString("Control", "MoveTo("+setPoint+")");
        return moveToInternal(setPoint.rotationValue);
    }

    /**
     * Move to set target. {@link #moveTo(ElevatorSetPoint)} is prefered
     * @param target angle to move to
     * @return whether we have reached the target
     */
    public boolean moveTo(Angle target) {
        if (!m_callibrated) {
            return false;
        }
        ntPutString("Control", "MoveTo("+target+")");
        return moveToInternal(target);
    }

    private static final double epsilonRot = 0.1;

    private boolean moveToInternal(Angle angle) {
        if (!m_callibrated) {
            return false;
        }

        angle = clamp(angle, Constants.ELEVATOR_MIN, Constants.ELEVATOR_MAX);

        m_motor.setControl(getPooledMotionMagicVoltage(null, angle));

        return m_motor.m_wrapped.getPosition().getValue().minus(angle).abs(Units.Rotations) < epsilonRot;
    }

    /**
     * Move using a DutyCycleOut. Will actively hold position on low-valued output
     * @param rate how fast to move. Positive is up.
     */
    public void smartMoveRate(double rate) {
        rate = clamp(rate, -1., 1.);
        if (Math.abs(rate) < 0.001) {
            if (m_callibrated) {
                activeBrake();
            } else {
                ntPutString("Control", "StaticBrake");
                m_motor.setControl(STATIC_BRAKE);
                activeBrakeTarget = null;
            }
        } else {
            ntPutString("Control", "DutyCycleOut("+rate+")");

            // smart-limit the rate
            //*
            {
                double limit = 1.;
                final double motorRot = m_motor.m_wrapped.getPosition().getValue().in(Units.Rotations);
                final double topOuterBound = Constants.ELEVATOR_MAX.in(Units.Rotations);
                final double bottomOuterBound = Constants.ELEVATOR_MIN.in(Units.Rotations);
                final double buffer = Constants.ELEVATOR_SLOW_BUFFER.in(Units.Rotations);
                final double topInnerBound = topOuterBound - buffer;
                final double bottomInnerBound = bottomOuterBound + buffer;

                if (rate > 0. && motorRot >= topInnerBound) {
                    // 0 is at bottom (outside) of buffer, 1 is at top (inside) of buffer
                    final double factor = clamp((motorRot - topInnerBound) / buffer, 0., 1.);
                    limit = 1. - factor;
                } else if (rate < 0. && motorRot <= bottomInnerBound) {
                    // 0 is at top (outside) of buffer, 1 is at bottom (inside) of buffer
                    final double factor = clamp((bottomInnerBound - motorRot) / buffer, 0., 1.);
                    limit = 1. - factor;
                }

                ntPutNumber("smart move limit", limit);

                //rate = clamp(rate, -limit, limit);
                rate *= limit;
            } //*/

            m_motor.setControl(getPooledDutyCycleOut(null, rate));
            activeBrakeTarget = null;
        }
    }

    private void activeBrake() {
        if (activeBrakeTarget == null) {
            activeBrakeTarget = clamp(m_motor.m_wrapped.getPosition().getValue(), Constants.ELEVATOR_MIN, Constants.ELEVATOR_MAX);
        }
        ntPutString("Control", "ActiveBrake("+activeBrakeTarget+")");
        m_motor.setControl(getPooledMotionMagicVoltage(null, activeBrakeTarget));
    }

    public void moveToCalib() {
        if (!m_callibrated) {
            return;
        }
        activeBrakeTarget = null;
        ntPutString("Control", "MoveTo("+calibPos+")");

        m_motor.setControl(getPooledMotionMagicVoltage(null, Units.Rotations.of(calibPos)));
    }

    /**
     * Move downwards for callibration (rate dependent on warning switch)
     * Must be called periodically to move at the appropriate rate
     */
    public void moveDownForCallibration() {
        if (m_callibrated) return;

        double rate = inWarn() ? -0.1 : -0.2;
        activeBrakeTarget = null;
        moveRate(rate);
    }

    public SysIdRoutine createSysIdRoutine(double seconds) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.per(Units.Second).of(0.5),
                Units.Volts.of(2.0),
                Units.Seconds.of(seconds)
            ),
            new SysIdRoutine.Mechanism(
                voltage -> m_motor.setControl(new VoltageOut(voltage)),
                log -> {
                    var elevatorLogger = log.motor("elevatorSysId");
                    elevatorLogger.voltage(m_motor.m_wrapped.getMotorVoltage().getValue());
                    elevatorLogger.angularAcceleration(m_motor.m_wrapped.getAcceleration().getValue());
                    elevatorLogger.angularVelocity(m_motor.m_wrapped.getVelocity().getValue());
                    elevatorLogger.angularPosition(m_motor.m_wrapped.getPosition().getValue());
                },
                this
            )
        );
    }

    // Low-level motion methods

    /**
     * Move using a simple DutyCycleOut (i.e. no PID)
     * @param rate how fast to move. Positive is up.
     */
    public void moveRate(double rate) {
        ntPutString("Control", "DutyCycleOut("+rate+")");
        m_motor.setControl(getPooledDutyCycleOut(null, rate));
        activeBrakeTarget = null;
    }

    public void stop() {
        ntPutString("Control", "StaticBrake");
        m_motor.setControl(STATIC_BRAKE);
        activeBrakeTarget = null;
    }

    public void disableStop() {
        ntPutString("Control", "StaticBrake");
        m_motor.setControl(STATIC_BRAKE);
        activeBrakeTarget = null;
    }

    protected boolean atBottom() {
        return !m_bottomLimitSwitch.get();
    }

    protected boolean inWarn() {
        return !m_warnLimitSwitch.get();
    }

    protected boolean atTop() {
        return m_topLimitSwitch.get();
    }
}
