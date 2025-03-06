package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Collects a group of commands to schedule in sequence.
 * 
 * Differs from {@link edu.wpi.first.wpilibj2.command.SequentialCommandGroup SequentialCommandGroup}
 * in that this will not hold on to the requirements of each child command.
 */
public class SequentialCommandScheduler extends Command {
    private final Command[] m_commands;
    private int m_current = -1;

    public SequentialCommandScheduler(Command... commands) {
        this.m_commands = commands;
    }

    private void scheduleNext() {
        if (m_current >= 0 && m_commands[m_current].isScheduled()) {
            m_commands[m_current].cancel();
        }
        m_current += 1;
        if (m_current < m_commands.length) {
            m_commands[m_current].schedule();
        }
    }

    @Override
    public void initialize() {
        m_current = -1;
    }

    @Override
    public void execute() {
        if (m_current >= m_commands.length) {
            return;
        }

        if (m_current < 0 || !m_commands[m_current].isScheduled()) {
            scheduleNext();
        }
    }

    @Override
    public boolean isFinished() {
        return m_current >= m_commands.length;
    }

    @Override
    public void end(boolean interrupted) {
        if (m_current >= 0 && m_current < m_commands.length) {
            m_commands[m_current].cancel();
        }
    }
}
