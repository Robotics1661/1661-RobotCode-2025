package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ClimberSubsystem extends CustomSubsystemBase {
    private final TalonFX m_motor = new TalonFX(Constants.CLIMBER_MOTOR);

    public ClimberSubsystem() {
        setSubsystem("climber");

        var conf = m_motor.getConfigurator();
        conf.apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
        );
    }

    // Motion methods

    private static double speed = 0.2;

    public void extend() {
        m_motor.setControl(getPooledVoltageOut(null, Units.Volts.of(12. * speed)));
    }

    public void retract() {
        m_motor.setControl(getPooledVoltageOut(null, Units.Volts.of(-12. * speed)));
    }

    public void stop() {
        m_motor.setControl(STATIC_BRAKE);
    }
}
