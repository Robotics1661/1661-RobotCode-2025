package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final DoubleSupplier m_directionInput;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier directionInput) {
        this.m_elevator = elevator;
        this.m_directionInput = directionInput;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        super.execute();

        m_elevator.smartMoveRate(m_directionInput.getAsDouble() * 0.4);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.disableStop();
        super.end(interrupted);
    }
}
