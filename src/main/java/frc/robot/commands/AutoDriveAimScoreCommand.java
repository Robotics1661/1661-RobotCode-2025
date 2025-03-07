package frc.robot.commands;

import static frc.robot.util.AutonomousDebugger.autoDbgCmd;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoint;
import frc.robot.subsystems.WristSubsystem.WristSetPoint;
import frc.robot.util.FieldPositioningUtils.ReefAlignment;

public class AutoDriveAimScoreCommand extends SequentialCommandGroup {
    public AutoDriveAimScoreCommand(
        CommandSwerveDrivetrain drivetrain,
        WristSubsystem wrist,
        ElevatorSubsystem elevator,
        IntakeSubsystem intake,
        double intake_rate,
        double intake_seconds,
        ReefAlignment reef_alignment,
        WristSetPoint wrist_set_point,
        ElevatorSetPoint elevator_set_point
    ) {
        addCommands(
            autoDbgCmd("Starting AutoAimScore(%s, %s, %s)".formatted(reef_alignment, wrist_set_point, elevator_set_point)),
            drivetrain.pathfindToNearestReef(
                reef_alignment,
                Units.Inches.of(8),
                Units.Inches.of(0.0),
                () -> new SequentialCommandGroup(
                    autoDbgCmd("> Starting ElevatorAndWristMoveTo"),
                    new ElevatorAndWristMoveToCommand(
                        elevator, wrist,
                        elevator_set_point, wrist_set_point
                    ),
                    autoDbgCmd("> Ending ElevatorAndWristMoveTo")
                ),
                wrist,
                elevator
            ),
            new IntakeAutoCommand(intake, intake_rate).withTimeout(intake_seconds),
            autoDbgCmd("> Starting back-off"),
            new ParallelCommandGroup(
                drivetrain.driveBack(Units.Meters.of(0.6)),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    autoDbgCmd("> Starting move down"),
                    new ElevatorAndWristMoveToCommand(
                        elevator, wrist,
                        ElevatorSetPoint.BASE, WristSetPoint.BASE
                    ).withAlignmentFactor(1.5)
                )
            ),
            autoDbgCmd("Finished AutoAimScore")
        );
    }
}
