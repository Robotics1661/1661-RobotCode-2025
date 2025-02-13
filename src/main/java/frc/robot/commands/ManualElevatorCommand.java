package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final BooleanSupplier m_goUp;
    private final BooleanSupplier m_goDown;

    public ManualElevatorCommand(ElevatorSubsystem elevator, BooleanSupplier goUp, BooleanSupplier goDown) {
        this.m_elevator = elevator;
        this.m_goUp = goUp;
        this.m_goDown = goDown;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        super.execute();

        if (m_goUp.getAsBoolean()) {
            m_elevator.moveRate(0.1);
        } else if (m_goDown.getAsBoolean()) {
            m_elevator.moveRate(-0.1);
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
