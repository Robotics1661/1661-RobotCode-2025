package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoint;

public class ElevatorManualCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final DoubleSupplier m_directionInput;
    private final BooleanSupplier m_goToTest;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier directionInput, BooleanSupplier goToTest) {
        this.m_elevator = elevator;
        this.m_directionInput = directionInput;
        this.m_goToTest = goToTest;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        super.execute();

        double control = m_directionInput.getAsDouble();

        if (m_goToTest.getAsBoolean()) {
            m_elevator.moveTo(ElevatorSetPoint.TESTING);
        } else if (Math.abs(control) > 0.1) {
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
