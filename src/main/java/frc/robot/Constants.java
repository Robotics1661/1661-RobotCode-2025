// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Unit;
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

    // Manipulator components
    public static final int MANIPULATOR_WRIST   = 22;
    public static final int MANIPULATOR_ENCODER = 23;
    public static final int MANIPULATOR_INTAKE  = 20;
    public static final Angle MANIPULATOR_WRIST_MIN = Units.Rotations.of(0.387);
    public static final Angle MANIPULATOR_WRIST_MAX = Units.Rotations.of(0.622);

    public static final boolean DISABLE_SWERVE = false; // not that swerve is implemented lol
    public static final boolean SKIP_INIT = true;
}
