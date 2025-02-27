package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ElevatorWristTestCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final ManipulatorSubsystem m_manipulator;

    private final BooleanSupplier m_elevatorUp;
    private final BooleanSupplier m_elevatorDown;
    private final BooleanSupplier m_wristUp;
    private final BooleanSupplier m_wristDown;

    public ElevatorWristTestCommand(
        ElevatorSubsystem elevator,
        ManipulatorSubsystem manipulator,
        BooleanSupplier elevatorUp,
        BooleanSupplier elevatorDown,
        BooleanSupplier wristUp,
        BooleanSupplier wristDown
    ) {
        this.m_elevator = elevator;
        this.m_manipulator = manipulator;
        this.m_elevatorUp = elevatorUp;
        this.m_elevatorDown = elevatorDown;
        this.m_wristUp = wristUp;
        this.m_wristDown = wristDown;

        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        if (m_elevatorUp.getAsBoolean()) {
            m_elevator.calibUp();
        } else if (m_elevatorDown.getAsBoolean()) {
            m_elevator.calibDown();
        }

        if (m_wristUp.getAsBoolean()) {
            m_manipulator.calibUp();
        } else if (m_wristDown.getAsBoolean()) {
            m_manipulator.calibDown();
        }

        m_elevator.moveToCalib();
        m_manipulator.moveToCalib();
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        m_manipulator.disableStop();
    }
}
