package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberManualCommand extends Command {
    private final ClimberSubsystem m_climber;
    private final BooleanSupplier m_lock_magnet;
    private final BooleanSupplier m_unlock_magnet;

    public ClimberManualCommand(ClimberSubsystem climber, BooleanSupplier lock_magnet, BooleanSupplier unlock_magnet) {
        this.m_climber = climber;
        this.m_lock_magnet = lock_magnet;
        this.m_unlock_magnet = unlock_magnet;

        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        if (m_lock_magnet.getAsBoolean()) {
            m_climber.lockMagnet();
        } else if (m_unlock_magnet.getAsBoolean()) {
            m_climber.unlockMagnet();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }
}
