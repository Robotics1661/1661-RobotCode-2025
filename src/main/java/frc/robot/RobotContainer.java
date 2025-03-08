// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.JoyXboxWrapper.BooleanButton;
import frc.robot.commands.AutoDriveAimScoreCommand;
import frc.robot.commands.ElevatorAndWristMoveToCommand;
import frc.robot.commands.ElevatorCallibrationCommand;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.IntakeManualCommand;
import frc.robot.commands.WristManualCommand;
import frc.robot.commands.WristMoveToCommand;
import frc.robot.commands.ElevatorMoveToCommand;
import frc.robot.commands.IntakeAutoCommand;
import frc.robot.commands.IntakeCoralAutoCommand;
import frc.robot.commands.testing.ElevatorSysIdCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoint;
import frc.robot.subsystems.WristSubsystem.WristSetPoint;
import frc.robot.util.CallibrationTracker;
import frc.robot.util.DoubleRingBuffer;
import frc.robot.util.FieldPositioningUtils;
import frc.robot.util.StringUtil;
import frc.robot.util.FieldPositioningUtils.ReefAlignment;
import frc.robot.util.FieldPositioningUtils.ReefHextant;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  // Drivetrain
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          // we handle deadband separately .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // average control out over `SMOOTH_PERIOD` inputs
  private static final int SMOOTH_PERIOD = 7;
  private final DoubleRingBuffer m_drive_forward_buffer = new DoubleRingBuffer(SMOOTH_PERIOD);
  private final DoubleRingBuffer m_drive_sideways_buffer = new DoubleRingBuffer(SMOOTH_PERIOD);

  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  //private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  // Controllers
  private final JoyXboxWrapper m_combined_controller = new JoyXboxWrapper(0, 3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Drivetrain
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivetrain.applyRequest(() -> drive
        .withVelocityX(m_drive_forward_buffer.smooth(-modifyAxis(m_combined_controller.getLateralY(), "dt_y", true) * MaxSpeed)) // Drive forward with negative Y (forward)
        .withVelocityY(m_drive_sideways_buffer.smooth(-modifyAxis(m_combined_controller.getLateralX(), "dt_x", true) * MaxSpeed)) // Drive left with negative X (left)
        .withRotationalRate(-modifyAxis(m_combined_controller.getRotation(), "dt_rot", true) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));

    m_combined_controller.X_BRAKE.getTrigger().whileTrue(m_drivetrain.applyRequest(() -> brake));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    /*joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

    // reset the field-centric heading
    m_combined_controller.ZERO_GYRO.getTrigger().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));
    m_combined_controller.ZERO_GYRO_180.getTrigger().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric180()));
    m_combined_controller.SET_POSE_FROM_LIMELIGHT.getTrigger().whileTrue(
      new RepeatCommand(
        new InstantCommand(
          () -> m_drivetrain.useLimelightPoseAnyway()
        ).ignoringDisable(true)
      ).ignoringDisable(true)
    );

    // Other things
    m_elevator.setDefaultCommand(new ElevatorManualCommand(
      m_elevator,
      deadband(m_combined_controller.ELEVATOR_CONTROL::get, 0.1)
    ));

    m_combined_controller.RESET_ELEVATOR.getTrigger().onTrue(
      new ElevatorMoveToCommand(m_elevator, ElevatorSetPoint.BASE)
      .until(JoyXboxWrapper.anyAbove(0.1, m_combined_controller.ELEVATOR_CONTROL))
    );

    m_wrist.setDefaultCommand(new WristManualCommand(
      m_wrist,
      deadband(m_combined_controller.WRIST_CONTROL::get, 0.1)
    ));

    m_intake.setDefaultCommand(new IntakeManualCommand(
      m_intake,
      m_combined_controller.INTAKE_CORAL::get,
      m_combined_controller.INTAKE_ALGAE::get,
      m_combined_controller.DROP_GAMEPIECE::get,
      m_combined_controller.DROP_SLOW::get
    ));

    // presets
    {
      registerPreset(m_combined_controller.PRESET_INTAKE, ElevatorSetPoint.INTAKE, WristSetPoint.INTAKE, true);
      m_combined_controller.PRESET_INTAKE.getTrigger()
      .and(m_combined_controller.DRIVE_TO_PRESET.getTrigger())
      .onTrue(m_drivetrain.pathfindToNearestIntakeAndPose().until(presetOverrideWithDrive));

      final var reefLeft = m_drivetrain.curryPathfindToNearestReef(
        ReefAlignment.LEFT,
        Units.Inches.of(8),
        Units.Inches.of(0.0)
      );
      final var reefCenter = m_drivetrain.curryPathfindToNearestReef(
        ReefAlignment.CENTER,
        Units.Inches.of(8),
        Units.Inches.of(0.0)
      );
      final var reefRight = m_drivetrain.curryPathfindToNearestReef(
        ReefAlignment.RIGHT,
        Units.Inches.of(8),
        Units.Inches.of(0.0)
      );

      registerPreset(m_combined_controller.PRESET_REEF_L1, ElevatorSetPoint.L1, WristSetPoint.L1, reefCenter);
      registerPreset(m_combined_controller.PRESET_REEF_LEFT_L2, ElevatorSetPoint.L2, WristSetPoint.L2, reefLeft);
      registerPreset(m_combined_controller.PRESET_REEF_RIGHT_L2, ElevatorSetPoint.L2, WristSetPoint.L2, reefRight);
      registerPreset(m_combined_controller.PRESET_REEF_LEFT_L3, ElevatorSetPoint.L3, WristSetPoint.L3, reefLeft);
      registerPreset(m_combined_controller.PRESET_REEF_RIGHT_L3, ElevatorSetPoint.L3, WristSetPoint.L3, reefRight);
      registerPreset(m_combined_controller.PRESET_REEF_LEFT_L4, ElevatorSetPoint.L4, WristSetPoint.L4, reefLeft);
      registerPreset(m_combined_controller.PRESET_REEF_RIGHT_L4, ElevatorSetPoint.L4, WristSetPoint.L4, reefRight);
      
      // algae
      final Supplier<Command> poseAlgae = () -> {
        ReefHextant hextant = ReefHextant.fromTranslation(m_drivetrain.getState().Pose.getTranslation(), m_drivetrain.onRedAlliance());
        return new ElevatorAndWristMoveToCommand(
          m_elevator, m_wrist,
          hextant.highAlgae ? ElevatorSetPoint.ALGAE_HIGH : ElevatorSetPoint.ALGAE_LOW,
          hextant.highAlgae ? WristSetPoint.ALGAE_HIGH : WristSetPoint.ALGAE_LOW
        );
      };

      m_combined_controller.PRESET_REEF_ALGAE.getTrigger()
      .and(m_combined_controller.DRIVE_TO_PRESET.getTrigger().negate())
      .onTrue(new DeferredCommand(poseAlgae, Set.of(m_elevator, m_wrist)));

      m_combined_controller.PRESET_REEF_ALGAE.getTrigger()
      .and(m_combined_controller.DRIVE_TO_PRESET.getTrigger())
      .onTrue(reefCenter.apply(poseAlgae, new Subsystem[] {m_elevator, m_wrist}));
    }

    // triggers
    {
      registerPreset("Raise to Base", ElevatorSetPoint.BASE, WristSetPoint.BASE);
      registerPreset("Raise to Intake", ElevatorSetPoint.INTAKE, WristSetPoint.INTAKE);

      registerPreset("Raise to L1", ElevatorSetPoint.L1, WristSetPoint.L1);
      registerPreset("Raise to L2", ElevatorSetPoint.L2, WristSetPoint.L2);
      registerPreset("Raise to L3", ElevatorSetPoint.L3, WristSetPoint.L3);
      registerPreset("Raise to L4", ElevatorSetPoint.L4, WristSetPoint.L4);

      registerNamedDriveAimScore(
        "Score L1",
        ReefAlignment.CENTER,
        ElevatorSetPoint.L1,
        WristSetPoint.L1,
        -0.07,
        2.0
      );
      for (ReefAlignment alignment : ReefAlignment.LEFT_RIGHT) {
        for (int L = 2; L <= 4; L++) {
          registerNamedDriveAimScore(
            StringUtil.titleCase("Score "+alignment.name()+" L"+L),
            alignment,
            ElevatorSetPoint.L(L),
            WristSetPoint.L(L),
            -0.25,
            0.75
          );
        }
      }

      NamedCommands.registerCommand("Intake Coral", new IntakeCoralAutoCommand(m_intake));

      new EventTrigger("Shoot Slow").onTrue(new IntakeAutoCommand(m_intake, -0.07).withTimeout(5.0));
    }

    /*m_climber.setDefaultCommand(Constants.DISABLE_CLIMB
      ? new EmptyDefaultCommand<>(m_climber)
      : new ClimberManualCommand(
        m_climber,
        m_combined_controller.CLIMB_LOCK::get,
        m_combined_controller.CLIMB_UNLOCK::get,
        m_combined_controller.CLIMB_UP::get,
        m_combined_controller.CLIMB_DOWN::get
    ));*/

    m_drivetrain.registerTelemetry(state -> logger.telemeterize(state, m_drivetrain.onRedAlliance()));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    new DebugField();

    System.out.println("Blue reef center: "+FieldPositioningUtils.blueReefCenter);
  }

  private void registerNamedDriveAimScore(
    String name,
    ReefAlignment reefAlignment,
    ElevatorSetPoint esp,
    WristSetPoint wsp,
    double intake_rate,
    double intake_seconds
  ) {
    System.out.println("Registering named command '"+name+"'");
    NamedCommands.registerCommand(
      name,
      new AutoDriveAimScoreCommand(
        m_drivetrain,
        m_wrist, m_elevator,
        m_intake,
        intake_rate, intake_seconds,
        reefAlignment,
        wsp, esp
      )
    );
  }

  private final BooleanSupplier presetOverride = JoyXboxWrapper.anyAbove(
    0.1,
    m_combined_controller.ELEVATOR_CONTROL,
    m_combined_controller.WRIST_CONTROL
  );

  private final BooleanSupplier presetOverrideWithDrive = JoyXboxWrapper.anyAbove(
    0.1,
    m_combined_controller.ELEVATOR_CONTROL,
    m_combined_controller.WRIST_CONTROL,
    () -> {
      double x = modifyAxis(m_combined_controller.getLateralY(), "dt_y", true);
      double y = modifyAxis(m_combined_controller.getLateralX(), "dt_x", true);

      return Math.sqrt(x*x + y*y) * 0.5;
    }
  );

  @SuppressWarnings("unused")
  private void registerPreset(
    BooleanButton button,
    ElevatorSetPoint esp,
    WristSetPoint wsp
  ) {
    registerPreset(button, esp, wsp, false);
  }

  private void registerPreset(
    BooleanButton button,
    ElevatorSetPoint esp,
    WristSetPoint wsp,
    boolean hasSeparateDrive
  ) {
    Trigger trigger = button.getTrigger();
    if (hasSeparateDrive) {
      trigger = trigger.and(m_combined_controller.DRIVE_TO_PRESET.getTrigger().negate());
    }
    trigger.onTrue(new ElevatorAndWristMoveToCommand(
      m_elevator, m_wrist,
      esp, wsp,
      presetOverride
    ));
  }

  private void registerPreset(
    BooleanButton button,
    ElevatorSetPoint esp,
    WristSetPoint wsp,
    BiFunction<Supplier<Command>, Subsystem[], Command> driveFactory 
  ) {
    button.getTrigger()
      .and(m_combined_controller.DRIVE_TO_PRESET.getTrigger().negate())
      .onTrue(new ElevatorAndWristMoveToCommand(
        m_elevator, m_wrist,
        esp, wsp
      ).until(presetOverride));

    button.getTrigger()
      .and(m_combined_controller.DRIVE_TO_PRESET.getTrigger())
      .onTrue(driveFactory.apply(
        () -> new ElevatorAndWristMoveToCommand(
          m_elevator, m_wrist,
          esp, wsp
        ),
        new Subsystem[] {m_elevator, m_wrist}
      ).until(presetOverrideWithDrive));
  }

  private void registerPreset(String eventName, ElevatorSetPoint esp, WristSetPoint wsp) {
    new EventTrigger(eventName).onTrue(new ElevatorAndWristMoveToCommand(m_elevator, m_wrist, esp, wsp));
    NamedCommands.registerCommand(eventName, new ElevatorAndWristMoveToCommand(m_elevator, m_wrist, esp, wsp));
  }

  public Command getSysIdCommand() {
    return ElevatorSysIdCommand.create(m_elevator, m_combined_controller.SYSID_SAFETY::get);
  }

  @SuppressWarnings("unused")
  public Command getInitCommand(boolean auto) {
    if (Constants.SKIP_INIT) {
      return null;
    } else {
      /*var climberCallibration = (m_climber.isCallibrated() || Constants.DISABLE_CLIMB)
        ? new InstantCommand()
        : new ClimberCallibrationCommand(m_climber);*/

      var elevatorCallibration = m_elevator.isCallibrated()
        ? new InstantCommand()
        : new ElevatorCallibrationCommand(m_elevator);

      /*var drivetrainCallibration = new InstantCommand(() -> m_drivetrain.runOnce(
        () -> m_drivetrain.seedFieldCentric()
      ));*/

      var wristCallibration = (m_wrist.withinSafeRange() || !auto || Utils.isSimulation())
        ? new InstantCommand()
        : new WristMoveToCommand(
          m_wrist,
          Constants.MANIPULATOR_WRIST_MAX.plus(Constants.MANIPULATOR_WRIST_MIN).div(2.0)
        ).andThen(new InstantCommand(() -> System.out.println("Wrist callibration complete")));

      return new SequentialCommandGroup(
        new InstantCommand(() -> CallibrationTracker.reset()),
        new ParallelCommandGroup(
          //climberCallibration,
          elevatorCallibration,
          //drivetrainCallibration,
          wristCallibration
        ),
        new InstantCommand(() -> CallibrationTracker.markCallibrated())
      );
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void stopAllOnDisable() {
    m_elevator.disableStop();
    m_wrist.disableStop();
    m_intake.disableStop();
    //m_climber.stop();
  }

  private static DoubleSupplier deadband(DoubleSupplier supplier, double deadband) {
    return () -> deadband(supplier.getAsDouble(), deadband);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyAxis(double value, String label, boolean square) {
    SmartDashboard.putNumber("controller_"+label+"_raw", value);
    // Deadband
    value = deadband(value, 0.1); // was 0.05
    SmartDashboard.putNumber("controller_"+label+"_db_cor", value);

    if (square) {
      // Square the axis
      value = Math.copySign(value * value, value);

      SmartDashboard.putNumber("controller_"+label+"_sq", value);
    }

    return value;
  }

  private class DebugField {
    private Pose2d testPose = new Pose2d();
    // field dimensions: 8.05 m by 17.55 m

    private static void addStaticPose(SendableBuilder builder, String name, Translation2d translation) {
      addStaticPose(builder, name, new Pose2d(translation, Rotation2d.kZero));
    }

    private static void addStaticPose(SendableBuilder builder, String name, Pose2d pose) {
      double[] poseArray = new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees()
      };
      builder.addDoubleArrayProperty(name,
        () -> poseArray,
        null
      );

      builder.addDoubleProperty(name+"X", () -> poseArray[0], null);
      builder.addDoubleProperty(name+"Y", () -> poseArray[1], null);
    }

    public DebugField() {
      SmartDashboard.putData("DebugField", builder -> {
        builder.setSmartDashboardType("Field2d");

        addStaticPose(builder, "blueReef", FieldPositioningUtils.blueReefCenter);
        addStaticPose(builder, "redReef", FieldPositioningUtils.redReefCenter);

        builder.addDoubleArrayProperty("test", () -> new double[] {
          testPose.getX(),
          testPose.getY(),
          testPose.getRotation().getDegrees()
        }, d -> testPose = new Pose2d(d[0], d[1], new Rotation2d(Units.Degrees.of(d[2]))));

        builder.addDoubleProperty(
          "testX",
          () -> testPose.getX(),
          x -> testPose = new Pose2d(x, testPose.getY(), testPose.getRotation())
        );

        builder.addDoubleProperty(
          "testY",
          () -> testPose.getY(),
          y -> testPose = new Pose2d(testPose.getX(), y, testPose.getRotation())
        );
      });
    }
  }
}
