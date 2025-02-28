package frc.robot.commands.testing;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SimAutoCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public SimAutoCommand(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.applyRequest(() -> drive
            .withVelocityX(1.0 * MaxSpeed)
            .withVelocityY(0.1 * MaxSpeed)
        );
    }
}
