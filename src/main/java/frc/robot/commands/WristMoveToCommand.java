package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristMoveToCommand extends Command {
    private final WristSubsystem m_wrist;
    private final Angle m_target;

    public WristMoveToCommand(WristSubsystem wrist, Angle target) {
        this.m_wrist = wrist;
        this.m_target = target;
    }

    @Override
    public void execute() {
        if (m_wrist.moveWristTo(m_target)) {
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.stop();
    }
}
