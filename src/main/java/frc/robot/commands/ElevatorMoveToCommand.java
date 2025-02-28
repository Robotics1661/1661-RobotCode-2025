package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoint;

public class ElevatorMoveToCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final Angle m_target;

    public ElevatorMoveToCommand(ElevatorSubsystem elevator, ElevatorSetPoint target) {
        this(elevator, target.rotationValue);
    }

    public ElevatorMoveToCommand(ElevatorSubsystem elevator, Angle target) {
        this.m_elevator = elevator;
        this.m_target = target;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        if (m_elevator.moveTo(m_target)) {
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }
}
