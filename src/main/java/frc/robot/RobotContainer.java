// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoytick = new Joystick(GamepadJoystick.kControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(GamepadJoystick.kDriverYAxis),
      () -> driverJoytick.getRawAxis(GamepadJoystick.kDriverXAxis),
      () -> driverJoytick.getRawAxis(GamepadJoystick.kDriverRotAxis),
      () -> !driverJoytick.getRawButton(GamepadJoystick.kDriverFieldOrientedButtonIdx)));

    configureBindings();
  }


  private void configureBindings() {
    new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
