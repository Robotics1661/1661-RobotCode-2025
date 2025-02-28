package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoint;
import frc.robot.subsystems.WristSubsystem.WristSetPoint;

public class ElevatorAndWristMoveToCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;

    private final ElevatorSetPoint m_elevatorSetPoint;
    private final WristSetPoint m_wristSetPoint;

    private final BooleanSupplier m_cancelSupplier;

    private boolean done = false;

    public ElevatorAndWristMoveToCommand(
        ElevatorSubsystem elevator,
        WristSubsystem wrist,
        ElevatorSetPoint elevatorSetPoint,
        WristSetPoint wristSetPoint,
        BooleanSupplier cancelSupplier
    ) {
        this.m_elevator = elevator;
        this.m_wrist = wrist;
        this.m_elevatorSetPoint = elevatorSetPoint;
        this.m_wristSetPoint = wristSetPoint;
        this.m_cancelSupplier = cancelSupplier;

        addRequirements(m_elevator, m_wrist);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (m_cancelSupplier.getAsBoolean()) {
            cancel();
            return;
        }

        boolean elevatorDone = m_elevator.moveTo(m_elevatorSetPoint);
        boolean wristDone = m_wrist.moveWristTo(m_wristSetPoint);
        if (elevatorDone && wristDone) {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_wrist.stop();
            m_elevator.stop();
        } else {
            m_wrist.moveRate(0);    // keep active brake set point
            m_elevator.moveRate(0); // keep active brake set point
        }
    }
}
