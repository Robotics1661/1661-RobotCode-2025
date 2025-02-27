package frc.robot.subsystems;

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
        calibPos += 2.0;
    }

    public void calibDown() {
        calibPos -= 2.0;
    }

    public enum ElevatorSetPoint {
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

    public void moveTo(ElevatorSetPoint setPoint) {
        if (!m_callibrated) {
            return;
        }
        ntPutString("Control", "MoveTo("+setPoint+")");

        m_motor.setControl(getPooledMotionMagicVoltage(null, setPoint.rotationValue));
        //m_motor.setControl(getPooledPositionVoltage(null, setPoint.rotationValue));
    }

    public void moveToCalib() {
        if (!m_callibrated) {
            return;
        }
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
    }

    public void stop() {
        ntPutString("Control", "StaticBrake");
        m_motor.setControl(STATIC_BRAKE);
    }

    public void disableStop() {
        ntPutString("Control", "StaticBrake");
        m_motor.setControl(STATIC_BRAKE);
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
