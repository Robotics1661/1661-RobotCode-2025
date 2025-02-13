package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCallibrationCommand extends Command {
    private final ElevatorSubsystem m_elevator;

    public ElevatorCallibrationCommand(ElevatorSubsystem elevator) {
        this.m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.startCallibration();
    }

    @Override
    public void execute() {
        m_elevator.moveDownForCallibration();
    }

    @Override
    public boolean isFinished() {
        return m_elevator.completeCallibration();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_elevator.disableStop();
        } else {
            m_elevator.stop();
        }
    }
}
