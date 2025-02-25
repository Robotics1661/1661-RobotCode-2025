package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ClimberSubsystem extends CustomSubsystemBase {
    private final DigitalInput m_magnet_limit = new DigitalInput(Constants.CLIMBER_LIMIT_MAGNET_OFF);
    private final SparkMax m_magnet_motor = new SparkMax(Constants.CLIMBER_MAGNET, MotorType.kBrushless);
    private final SparkMax m_climb_motor = new SparkMax(Constants.CLIMBER_VERTICAL, MotorType.kBrushed);

    private CallibrationState m_callibration_state = CallibrationState.UNCALLIBRATED;

    public ClimberSubsystem() {
        setSubsystem("climber");

        configureMagnetMotor();

        ntPutData(builder -> {
            builder.addStringProperty("Callibrated", () -> m_callibration_state.name(), null);
            builder.addBooleanProperty("At Off", this::isAtOff, null);
            builder.addDoubleProperty("Encoder", () -> m_magnet_motor.getEncoder().getPosition(), null);
        });
    }

    private void configureMagnetMotor() {
        var magnet_config = new SparkMaxConfig();
        magnet_config.apply(new SoftLimitConfig()
            .forwardSoftLimit(28.0)
            .reverseSoftLimit(0.0)
            .forwardSoftLimitEnabled(m_callibration_state.complete)
            .reverseSoftLimitEnabled(m_callibration_state.complete)
        );
        m_magnet_motor.configure(
            magnet_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    @Override
    public void periodic() {
        if (m_callibration_state == CallibrationState.IN_PROGRESS && isAtOff()) {
            m_magnet_motor.stopMotor();
            m_magnet_motor.getEncoder().setPosition(0.0);

            m_callibration_state = CallibrationState.CALLIBRATED;

            configureMagnetMotor();

            log("Callibration complete");
        } else if (m_callibration_state == CallibrationState.CALLIBRATED) {
            if (m_magnet_motor.get() > 0.0 && m_magnet_motor.getEncoder().getPosition() >= 27.5) {
                m_magnet_motor.stopMotor();
            } else if (m_magnet_motor.get() < 0.0 && m_magnet_motor.getEncoder().getPosition() <= 2.0) {
                m_magnet_motor.stopMotor();
            }
        }
    }

    // Motion methods

    public void lockMagnet() {
        if (m_callibration_state.complete) {
            m_magnet_motor.set(0.3);
        }
    }

    public void unlockMagnet() {
        if (m_callibration_state.complete) {
            m_magnet_motor.set(-0.3);
        }
    }

    public void stop() {
        if (m_callibration_state == CallibrationState.IN_PROGRESS) {
            m_callibration_state = CallibrationState.UNCALLIBRATED;
        }
        m_magnet_motor.stopMotor();
        m_climb_motor.stopMotor();
    }

    // Callibration state

    public void startCallibration() {
        log("Starting callibration");
        m_magnet_motor.stopMotor();
        m_callibration_state = CallibrationState.IN_PROGRESS;
        configureMagnetMotor();
        m_magnet_motor.set(-0.06);
    }

    public boolean isCallibrated() {
        return m_callibration_state.complete;
    }

    private boolean isAtOff() {
        return !m_magnet_limit.get();
    }

    private enum CallibrationState {
        UNCALLIBRATED(false),
        IN_PROGRESS(false),
        CALLIBRATED(true)
        ;
        public final boolean complete;

        CallibrationState(boolean complete) {
            this.complete = complete;
        }
    }
}
