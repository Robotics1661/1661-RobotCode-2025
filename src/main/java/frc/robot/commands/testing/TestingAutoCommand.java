package frc.robot.commands.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// field dimensions: 8.05 m by 17.55 m
public class TestingAutoCommand {
    public static Command create(CommandSwerveDrivetrain drivetrain) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPose(new Pose2d(17.55/2, 7.0, new Rotation2d())), drivetrain),
            drivetrain.pathfindToNearestIntake()
        );
    }
}
