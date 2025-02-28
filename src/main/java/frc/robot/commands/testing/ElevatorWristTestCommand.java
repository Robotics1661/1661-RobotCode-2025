package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ElevatorWristTestCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;

    public ElevatorWristTestCommand(
        ElevatorSubsystem elevator,
        WristSubsystem wrist
    ) {
        this.m_elevator = elevator;
        this.m_wrist = wrist;

        addRequirements(m_elevator, m_wrist);
    }

    @Override
    public void execute() {
        m_elevator.moveToCalib();
        m_wrist.moveToCalib();
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        m_wrist.disableStop();
    }
}
