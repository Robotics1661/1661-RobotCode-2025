package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorSubsystem extends CustomSubsystemBase {
    private final TalonFX m_motor = new TalonFX(Constants.ELEVATOR_MOTOR);
    private final DigitalInput m_topLimitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_TOP);
    private final DigitalInput m_warnLimitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_WARN);
    private final DigitalInput m_bottomLimitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_BOTTOM);

    /** Whether we've zeroed to the bottom yet */
    private boolean m_callibrated = false;

    public ElevatorSubsystem() {
        setSubsystem("elevator");

        // configure the motor to behave itself properly
        var motorConfig = m_motor.getConfigurator();
        motorConfig.apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) // we want positive to correspond to up
            .withNeutralMode(NeutralModeValue.Brake) // don't just slide down when nothing is being output
        );

        m_motor.setControl(new CoastOut());
    }

    @Override
    public void periodic() {
        // since we're using limit switches for software-stop, we need to send their state to the motor every tick
        // IMPORTANT: if we start using any other types of control requests, this will need to be applied to them too
        if (m_motor.getAppliedControl() instanceof DutyCycleOut dco) {
            boolean shouldLimitForward = m_topLimitSwitch.get();
            boolean shouldLimitReverse = m_bottomLimitSwitch.get();

            // however, there's no point flooding the CAN bus if nothing has changed.
            if (dco.LimitForwardMotion != shouldLimitForward || dco.LimitReverseMotion != shouldLimitReverse) {
                m_motor.setControl(dco
                .withLimitForwardMotion(shouldLimitForward)
                .withLimitReverseMotion(shouldLimitReverse));
            }
        }
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
        if (!m_bottomLimitSwitch.get()) {
            return false;
        }

        stop();
        m_motor.setPosition(0);
        m_callibrated = true;

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

        double rate = m_warnLimitSwitch.get() ? -0.1 : -0.2;
        moveRate(rate);
    }

    // Low-level motion methods

    /**
     * Move using a simple DutyCycleOut (i.e. no PID)
     * @param rate how fast to move. Positive is up.
     */
    public void moveRate(double rate) {
        m_motor.setControl(new DutyCycleOut(rate)
        .withLimitForwardMotion(m_topLimitSwitch.get())
        .withLimitReverseMotion(m_bottomLimitSwitch.get()));
    }

    public void stop() {
        m_motor.setControl(new StaticBrake());
    }

    public void disableStop() {
        m_motor.setControl(new CoastOut());
    }
}
