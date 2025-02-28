package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristMoveToCommand extends Command {
    private final WristSubsystem m_wrist;
    private final Angle m_target;

    private boolean done = false;

    public WristMoveToCommand(WristSubsystem wrist, Angle target) {
        this.m_wrist = wrist;
        this.m_target = target;

        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (m_wrist.moveWristTo(m_target)) {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.stop();
    }
}
