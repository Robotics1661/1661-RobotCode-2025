package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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

    public ElevatorSubsystem() {
        setSubsystem("elevator");

        // configure the motor to behave itself properly
        var motorConfig = m_motor.m_wrapped.getConfigurator();
        motorConfig.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // we want positive to correspond to up
            .withNeutralMode(NeutralModeValue.Brake) // don't just slide down when nothing is being output
        );

        m_motor.setControl(COAST_OUT);

        ntPutString("Control", "None");
        ntPutString("Position", "Unknown");
        ntPutBoolean("Callibrated", false);
    }

    @Override
    public void periodic() {
        ntPutBoolean("At Top", atTop());
        ntPutBoolean("In Bottom Warn", inWarn());
        ntPutBoolean("At Bottom", atBottom());
        ntPutString("Position", m_motor.m_wrapped.getPosition().toString());
        ntPutBoolean("Callibrated", m_callibrated);

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
        if (!atBottom()) {
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

    /**
     * Move downwards for callibration (rate dependent on warning switch)
     * Must be called periodically to move at the appropriate rate
     */
    public void moveDownForCallibration() {
        if (m_callibrated) return;

        double rate = inWarn() ? -0.1 : -0.2;
        moveRate(rate);
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
        ntPutString("Control", "CoastOut");
        m_motor.setControl(COAST_OUT);
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
