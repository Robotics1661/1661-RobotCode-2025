// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ClimberCallibrationCommand;
import frc.robot.commands.ClimberManualCommand;
import frc.robot.commands.ElevatorCallibrationCommand;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.ElevatorSysIdCommand;
import frc.robot.commands.IntakeManualCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // Controllers
  private final JoyXboxWrapper m_combined_controller = new JoyXboxWrapper(0, 3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_elevator.setDefaultCommand(new ElevatorManualCommand(
      m_elevator,
      m_combined_controller.ELEVATOR_CONTROL::get,
      m_combined_controller.ELEVATOR_TEST::get
    ));

    m_manipulator.setDefaultCommand(new IntakeManualCommand(
      m_manipulator,
      m_combined_controller.INTAKE_CORAL::get,
      m_combined_controller.DROP_GAMEPIECE::get,
      m_combined_controller.WRIST_TEST::get,
      m_combined_controller.WRIST_CONTROL::get
    ));

    m_climber.setDefaultCommand(new ClimberManualCommand(
      m_climber,
      m_combined_controller.CLIMB_LOCK::get,
      m_combined_controller.CLIMB_UNLOCK::get
    ));

    //m_combined_controller.CLIMB_DOWN.getTrigger().onTrue(new ClimberCallibrationCommand(m_climber));
    m_combined_controller.ELEVATOR_CALLIBRATE.getTrigger().onTrue(new ElevatorCallibrationCommand(m_elevator));
  }

  public Command getSysIdCommand() {
    return ElevatorSysIdCommand.create(m_elevator, m_combined_controller.SYSID_SAFETY::get);
  }

  @SuppressWarnings("unused")
  public Command getInitCommand() {
    if (m_elevator.isCallibrated() || Constants.SKIP_INIT) {
      return null;
    } else {
      return new ElevatorCallibrationCommand(m_elevator);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void stopAllOnDisable() {
    m_elevator.disableStop();
    m_manipulator.disableStop();
    m_climber.stop();
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
    value = deadband(value, m_combined_controller.isSoftDisabled() ? 0.2 : 0.1); // was 0.05
    SmartDashboard.putNumber("controller_"+label+"_db_cor", value);

    if (square) {
      // Square the axis
      value = Math.copySign(value * value, value);

      SmartDashboard.putNumber("controller_"+label+"_sq", value);
    }

    return value;
  }
}
