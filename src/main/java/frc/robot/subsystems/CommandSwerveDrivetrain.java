package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.util.PathfindThenRunThenFollowPath;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.util.FieldPositioningUtils;
import frc.robot.util.FieldPositioningUtils.ReefAlignment;
import frc.robot.util.FieldPositioningUtils.ReefHextant;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        customInit();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        customInit();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        customInit();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                m_isRed = allianceColor == Alliance.Red;
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        odometryUpdate();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    // Custom Code
    private boolean m_useLimelightPoseAnyway = false;
    private boolean m_isRed = false;

    private Pose2d $getPose() {
        return getState().Pose;
    }

    private ChassisSpeeds $getRobotRelativeSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private final SwerveRequest.ApplyRobotSpeeds $chassisSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private void $driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        setControl($chassisSpeedsRequest.withSpeeds(chassisSpeeds));
    }

    public boolean onRedAlliance() {
        return m_isRed;
    }

    public boolean shouldFlipPath() {
        return m_isRed;
    }

    private void customInit() {
        AutoBuilder.configure(
            this::$getPose,
            this::resetPose,
            this::$getRobotRelativeSpeeds,
            this::$driveRobotRelative,
            $makeController(),
            Constants.PATHPLANNER_CONFIG,
            this::shouldFlipPath,
            this
        );
    }

    private PathFollowingController $makeController() {
        return new PPHolonomicDriveController( // based on nothing. at all.
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)
        );
    }

    private PathConstraints $makePathConstraints() {
        return new PathConstraints(
            4.0, 4.0,
            Math.toRadians(540), Math.toRadians(720)
        );
    }

    public Command pathfindTo(PathPlannerPath targetPath, Supplier<Command> runAfterPathfind, Subsystem... extraRequirements) {
        Set<Subsystem> requirements = new HashSet<>();
        for (Subsystem req : extraRequirements) {
            requirements.add(req);
        }
        requirements.add(this);
        return new PathfindThenRunThenFollowPath(
            targetPath, runAfterPathfind,
            $makePathConstraints(),
            this::$getPose,
            this::$getRobotRelativeSpeeds,
            (cs, df) -> this.$driveRobotRelative(cs),
            $makeController(),
            Constants.PATHPLANNER_CONFIG,
            this::shouldFlipPath,
            requirements.toArray(new Subsystem[0])
        );
    }

    public Command pathfindTo(PathPlannerPath targetPath) {
        return AutoBuilder.pathfindThenFollowPath(
            targetPath,
            $makePathConstraints()
        );
    }

    public Command pathfindTo(Pose2d target) {
        return AutoBuilder.pathfindToPoseFlipped(
            target,
            $makePathConstraints()
        );
    }

    public Command pathfindToAbsolute(Pose2d target) {
        return AutoBuilder.pathfindToPose(
            target,
            $makePathConstraints()
        );
    }

    public Command pathfindToNearestIntakeAndPose() {
        return new DeferredCommand(
            () -> {
                try {
                    if (getState().Pose.getY() > 4.025 ^ shouldFlipPath()) {
                        return pathfindTo(PathPlannerPath.fromPathFile("Intake A"));
                    } else {
                        return pathfindTo(PathPlannerPath.fromPathFile("Intake B"));
                    }
                } catch (FileVersionException | IOException | ParseException e) {
                    e.printStackTrace();
                    return new InstantCommand();
                }
            },
            Set.of(this)
        );
    }

    public BiFunction<Supplier<Command>, Subsystem[], Command> curryPathfindToNearestReef(
        ReefAlignment alignment,
        Distance moveAt,
        Distance finalTarget
    ) {
        return (runAfterPathFind, extraRequirements) -> pathfindToNearestReef(
            alignment,
            moveAt,
            finalTarget,
            runAfterPathFind,
            extraRequirements
        );
    }

    public Command pathfindToNearestReef(
        ReefAlignment alignment,
        Distance moveAt,
        Distance finalTarget,
        Supplier<Command> runAfterPathFind,
        Subsystem... extraRequirements
    ) {
        return new DeferredCommand(
            () -> {
                ReefHextant hextant = ReefHextant.fromTranslation(getState().Pose.getTranslation(), m_isRed);

                final var baseRadius = FieldPositioningUtils.reefInnerRadius.plus(FieldPositioningUtils.robotInnerRadius);
                final var approachRadius = baseRadius.plus(moveAt);
                final var targetRadius = baseRadius.plus(finalTarget);
                final var perpendicular = Units.Inches.of(6.47 * alignment.sign);

                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    hextant.makePose(approachRadius, perpendicular, m_isRed),
                    hextant.makePose(targetRadius, perpendicular, m_isRed)
                );

                PathConstraints constraints = new PathConstraints(
                    3.0, 4.0,
                    Math.toRadians(540), Math.toRadians(720)
                );

                PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    new IdealStartingState(0.0, hextant.facing(m_isRed)),
                    new GoalEndState(0.0, hextant.facing(m_isRed))
                );
                path.preventFlipping = true;
                if (m_isRed) {
                    path = path.flipPath();
                    path.preventFlipping = false;
                }

                return pathfindTo(path, runAfterPathFind, extraRequirements);
                /*return pathfindToAbsolute(hextant.makePose(
                    FieldPositioningUtils.reefInnerRadius
                        .plus(FieldPositioningUtils.robotInnerRadius)
                        .plus(Units.Inches.of(2.0)),
                    Units.Inches.of(6.47 * (left ? -1 : 1)),
                    m_isRed
                ));*/
            },
            Set.of(this)
        );
    }

    public Command driveBack(Distance distance) {
        return new DeferredCommand(
            () -> {
                final Pose2d currentPose = getState().Pose;
                final Rotation2d rotation = currentPose.getRotation();

                final Distance dx = distance.times(rotation.getCos());
                final Distance dy = distance.times(rotation.getSin());
                final Pose2d targetPose = new Pose2d(
                    currentPose.getMeasureX().minus(dx),
                    currentPose.getMeasureY().minus(dy),
                    rotation
                );

                return pathfindToAbsolute(targetPose);
            },
            Set.of(this)
        );
    }

    /**
     * Resets the rotation of the robot pose to 0 from the
     * {@link SwerveRequest.ForwardPerspectiveValue#OperatorPerspective}
     * perspective. This makes the current orientation of the robot X
     * forward for field-centric maneuvers.
     * <p>
     * This is equivalent to calling {@link #resetRotation} with the operator
     * perspective rotation.
     */
    public void seedFieldCentric180() {
        // intentionally swapped.
        resetRotation(m_isRed ? kBlueAlliancePerspectiveRotation : kRedAlliancePerspectiveRotation);
    }

    @SuppressWarnings("unused")
    private void odometryUpdate() {
        //System.out.println("Odometry update");
        if (Constants.ENABLE_LL_VISION_ESTIMATE && !Utils.isSimulation()) {
            updatePoseEstimatorWithVisionBotPose();
        }
    }

    public void updatePoseEstimatorWithVisionBotPose() {
        boolean useMegaTag2 = true;
        boolean doRejectUpdate = false;

        if (!useMegaTag2) { // MegaTag 1
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if (mt1 == null) {
                return;
            }

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                    SmartDashboard.putString("LL Status", "Single tag; Too ambiguous");
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                    SmartDashboard.putString("LL Status", "Single tag; Too far away");
                }
            }

            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
                SmartDashboard.putString("LL Status", "No tags");
            }

            double poseDifference = this.getState().Pose.getTranslation()
            .getDistance(mt1.pose.getTranslation());

            if (poseDifference > 1.) {
                if (m_useLimelightPoseAnyway) {
                    m_useLimelightPoseAnyway = false;
                } else {
                    doRejectUpdate = true;
                    SmartDashboard.putString("LL Status", "Too far off");
                }
            }

            if (!doRejectUpdate) {
                // use huge angle StdDev b/c we trust the gyro
                double xyStds = m_useLimelightPoseAnyway ? 0.01 : .5;
                setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 9999999));
                addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
                SmartDashboard.putString("LL Status", "Being integrated");
            }
        } else { // MegaTag 2
            LimelightHelpers.SetRobotOrientation( // most params are unnecesary for MT2
                "limelight",
                getState().Pose.getRotation().getDegrees(), 0,
                0, 0,
                0, 0
            );
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            if (mt2 == null) {
                return;
            }

            // reject vision updates if rotating faster than 720 deg/sec
            if (Math.abs(getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(720)) {
                doRejectUpdate = true;
                SmartDashboard.putString("LL Status", "Rotating too fast");
            }

            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
                SmartDashboard.putString("LL Status", "No tags");
            }

            double poseDifference = this.getState().Pose.getTranslation()
            .getDistance(mt2.pose.getTranslation());

            if (poseDifference > 1.) {
                if (m_useLimelightPoseAnyway) {
                    m_useLimelightPoseAnyway = false;
                } else {
                    doRejectUpdate = true;
                    SmartDashboard.putString("LL Status", "Too far off");
                }
            }

            if (!doRejectUpdate) {
                double xyStds = m_useLimelightPoseAnyway ? 0.01 : .7;
                setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 9999999));
                addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                SmartDashboard.putString("LL Status", "Being integrated");
            }
        }
        /*PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
        // invalid LL data
        if (visionBotPose.pose2d().getX() == 0.0) {
            System.out.println("[LL] Invalid LL data");
            SmartDashboard.putString("LL Status", "Invalid LL data");
            return;
        }

        SmartDashboard.putNumber("LL Last Update", Utils.getCurrentTimeSeconds());

        // distance from current pose to vision estimated pose
        double poseDifference = this.getState().Pose.getTranslation()
            .getDistance(visionBotPose.pose2d().getTranslation());

        if (m_visionSystem.areAnyTargetsValid()) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (m_visionSystem.getNumberOfTargetsVisible() >= 2) {
                xyStds = 0.5;
                degStds = 6;
                //System.out.println("[LL] >= 2 targets");
                SmartDashboard.putString("LL Status", ">= 2 targets");
            }
            // 1 target with large area and close to estimated pose
            else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
                //System.out.println("[LL] 1 big target");
                SmartDashboard.putString("LL Status", "1 big target");
            }
            // 1 target farther away and estimated pose is close
            else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
                //System.out.println("[LL] 1 small target");
                SmartDashboard.putString("LL Status", "1 small target");
            }
            else if (m_useLimelightPoseAnyway) {
                m_useLimelightPoseAnyway = false;
                xyStds = 0;
                degStds = 0;
                System.out.println("[LL] Using limelight pose anyway");
                SmartDashboard.putString("LL Status", "Using pose anyway");
            }
            // conditions don't match to add a vision measurement
            else {
                //System.out.println("[LL] Too large pose difference: "+poseDifference);
                SmartDashboard.putString("LL Status", "Too large pose difference: "+poseDifference);
                return;
            }

            setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds)));
            addVisionMeasurement(visionBotPose.pose2d(),
                visionBotPose.timestampSeconds());
        } else {
            SmartDashboard.putString("LL Status", "No valid targets");
        }*/
    }

    public void useLimelightPoseAnyway() {
        System.out.println("Will use limelight pose anyway");
        m_useLimelightPoseAnyway = true;
    }
}
