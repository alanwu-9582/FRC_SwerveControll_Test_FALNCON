// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCmd extends CommandBase {
  private boolean isClimbing = false;
  private boolean isEngaged = false;
  private SwerveSubsystem swerveSubsystem;

  public AutoBalanceCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isClimbing = false;
    isEngaged = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isEngaged) {
      this.swerveSubsystem.lockModules();
    } else {
      double pitch = this.swerveSubsystem.getPitch();
      if (isClimbing) {
        double climbSpeed = Math.sin(pitch);
        this.swerveSubsystem.move(0.0, climbSpeed, 0.0, false);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
