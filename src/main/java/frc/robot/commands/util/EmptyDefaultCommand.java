package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class EmptyDefaultCommand<T extends Subsystem> extends Command {
    private final T m_subsystem;

    public EmptyDefaultCommand(T subsystem) {
        this.m_subsystem = subsystem;

        addRequirements(m_subsystem);
    }
}
