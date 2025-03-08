package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoint;
import frc.robot.subsystems.WristSubsystem.WristSetPoint;
import frc.robot.util.AutonomousDebugger;

public class ElevatorAndWristMoveToCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;

    private final ElevatorSetPoint m_elevatorSetPoint;
    private final WristSetPoint m_wristSetPoint;

    private final BooleanSupplier m_cancelSupplier;

    private double m_alignmentFactor = 1.0;

    private boolean done = false;

    private Timer startTime;

    public ElevatorAndWristMoveToCommand(
        ElevatorSubsystem elevator,
        WristSubsystem wrist,
        ElevatorSetPoint elevatorSetPoint,
        WristSetPoint wristSetPoint
    ) {
        this(
            elevator,
            wrist,
            elevatorSetPoint,
            wristSetPoint,
            () -> false
        );
    }

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

    public ElevatorAndWristMoveToCommand withAlignmentFactor(double alignmentFactor) {
        this.m_alignmentFactor = alignmentFactor;
        return this;
    }

    @Override
    public void initialize() {
        done = false;
        startTime = new Timer();
        startTime.start();

        if (Utils.isSimulation()) {
            AutonomousDebugger.autoDbg(toString() + " started");
        }
    }

    @Override
    public void execute() {
        if (m_cancelSupplier.getAsBoolean()) {
            cancel();
            return;
        }

        if (Utils.isSimulation()) {
            if (startTime.hasElapsed(2.0)) {
                done = true;
            }
            SmartDashboard.putBoolean("ElevatorWristMoveTo/elevatorDone", done);
            SmartDashboard.putBoolean("ElevatorWristMoveTo/wristDone", done);
        } else {
            boolean elevatorDone = m_elevator.moveTo(m_elevatorSetPoint, m_alignmentFactor);
            boolean wristDone = m_wrist.moveWristTo(m_wristSetPoint, m_alignmentFactor);
            SmartDashboard.putBoolean("ElevatorWristMoveTo/elevatorDone", elevatorDone);
            SmartDashboard.putBoolean("ElevatorWristMoveTo/wristDone", wristDone);
            if (elevatorDone && wristDone) {
                done = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            AutonomousDebugger.autoDbg(toString() + " interrupted");
            m_wrist.stop();
            m_elevator.stop();
        } else {
            AutonomousDebugger.autoDbg(toString() + " ended naturally");
            m_wrist.moveRate(0);    // keep active brake set point
            m_elevator.moveRate(0); // keep active brake set point
        }
    }

    @Override
    public String toString() {
        return getClass().getSimpleName() + "(" + m_elevatorSetPoint + ", " + m_wristSetPoint + ")";
    }
}
