package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCallibrationCommand extends Command {
    private final ClimberSubsystem m_climber;

    public ClimberCallibrationCommand(ClimberSubsystem climber) {
        this.m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.startCallibration(); // climber handles callibration in Periodic
    }

    @Override
    public boolean isFinished() {
        return m_climber.isCallibrated();
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }
}
