# 1661-RobotCode-2025

Java robot code for the 2025 season (REEFSCAPE) from [Griffitrons 1661](https://team1661.com), the FIRST Robotics Competition team at The Buckley School.

## Features

- **Swerve drivetrain** on CTRE Phoenix 6 using Tuner X generated constants (`CommandSwerveDrivetrain`)
- **Elevator, wrist, intake and climber** subsystems, with manual, move-to and calibration commands
- Automatic coral intake, plus an auto drive, aim and score command
- **PathPlanner** autos, including a command that pathfinds, runs a command, then follows a path
- Limelight vision helpers (`limelight/LimelightHelpers.java`)
- SysId characterization commands and an autonomous debugger for testing and tuning
- Drivetrain telemetry (`Telemetry.java`)

## Layout

```
src/main/java/frc/robot/
├── subsystems/   # drivetrain, elevator, wrist, intake, climber
├── commands/     # teleop and autonomous commands (plus util/ and testing/)
├── generated/    # Tuner X swerve constants
├── limelight/    # Limelight helpers
└── util/         # math, calibration and logging helpers
```

Vendor libraries: Phoenix 6, PathPlanner and REVLib (see `vendordeps/`).

## Building and running

You'll need [WPILib 2025](https://docs.wpilib.org/) (it bundles the JDK and Gradle). Then, from the repo root:

```bash
./gradlew build          # compile and run checks
./gradlew simulateJava   # run in the WPILib simulator
./gradlew deploy         # deploy to a roboRIO (must be connected to the robot)
```

On Windows use `gradlew.bat` instead of `./gradlew`. The team number (1661) lives in
`.wpilib/wpilib_preferences.json`.

## License

Team-written code is released under the [MIT License](LICENSE). Files generated from the WPILib
project template keep their original headers, and WPILib itself is covered by the BSD-style license in
[WPILib-License.md](WPILib-License.md). Vendor libraries in `vendordeps/` are covered by their own licenses.
