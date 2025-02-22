package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final DoubleSupplier m_directionInput;

    public ManualElevatorCommand(ElevatorSubsystem elevator, DoubleSupplier directionInput) {
        this.m_elevator = elevator;
        this.m_directionInput = directionInput;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        super.execute();

        double control = m_directionInput.getAsDouble();

        if (Math.abs(control) > 0.1) {
            m_elevator.moveRate(control * 0.2);
        } else {
            m_elevator.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.disableStop();
        super.end(interrupted);
    }
}
