// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Elevator components
    public static final int ELEVATOR_MOTOR        = 24;
    public static final int ELEVATOR_LIMIT_TOP    = 0; // DIO
    public static final int ELEVATOR_LIMIT_WARN   = 1; // DIO
    public static final int ELEVATOR_LIMIT_BOTTOM = 2; // DIO

    public static final Angle ELEVATOR_MAX = Units.Rotations.of(76.0);
    public static final Angle ELEVATOR_MIN = Units.Rotations.zero();
    public static final Angle ELEVATOR_SLOW_BUFFER = Units.Rotations.of(10.0);

    // Manipulator components
    public static final int MANIPULATOR_WRIST   = 22;
    public static final int MANIPULATOR_ENCODER = 23;
    public static final int MANIPULATOR_INTAKE  = 20;
    public static final Angle MANIPULATOR_WRIST_MIN = Units.Rotations.of(0.395);
    public static final Angle MANIPULATOR_WRIST_MAX = Units.Rotations.of(0.622);
    public static final Angle MANIPULATOR_WRIST_SLOW_BUFFER = Units.Rotations.of(0.05);

    // Climber components
    public static final int CLIMBER_MAGNET           = 25;
    public static final int CLIMBER_VERTICAL         = 21;
    public static final int CLIMBER_LIMIT_MAGNET_OFF = 3; // DIO

    // Drivetrain
    public static final RobotConfig PATHPLANNER_CONFIG = getRobotConfig();
    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    // Flags
    public static final boolean DISABLE_SWERVE = false;
    public static final boolean DISABLE_CLIMB = true;

    public static final boolean ENABLE_LL_VISION_ESTIMATE = false; // TODO update limelight firmware & verify this is safe

    public static final boolean SKIP_INIT = false;
    public static final boolean USE_CONTROLS_IN_SIM = true;
}
