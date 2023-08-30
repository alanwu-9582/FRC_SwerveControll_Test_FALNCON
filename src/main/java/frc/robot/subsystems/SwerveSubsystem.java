// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.robotMap;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
    robotMap.kFrontLeftDriveMotorPort,
    robotMap.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    robotMap.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule frontRight = new SwerveModule(
    robotMap.kFrontRightDriveMotorPort,
    robotMap.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    robotMap.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(
    robotMap.kBackLeftDriveMotorPort,
    robotMap.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    robotMap.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(
    robotMap.kBackRightDriveMotorPort,
    robotMap.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    robotMap.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDrivePoseEstimator SwerveEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
    frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
  }, new Pose2d());
  private final Field2d field = new Field2d();

  public SwerveSubsystem() {
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
            resetEncoders();
        } catch (Exception e) {
        }
    }).start();
  }

  public void zeroHeading() {
      gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return SwerveEstimator.getEstimatedPosition();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public void resetOdometry(Pose2d pose) {
    SwerveEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}, pose);
  }

  @Override
  public void periodic() {
      SwerveEstimator.update(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
      // var gloabalPose = vision.getEstimatedGlobalPose();
      // if (vision.hasTarget()) SwerveEstimator.addVisionMeasurement(gloabalPose.get().getFirst(), gloabalPose.get().getSecond());
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putNumber("Robot Pitch", getPitch());
      SmartDashboard.putData(field);
      field.setRobotPose(getPose());
      backLeft.putDashboard();
      backRight.putDashboard();
      frontLeft.putDashboard();
      frontRight.putDashboard();
  }

  public void stopModules() {
      frontLeft.stop();
      frontRight.stop();
      backLeft.stop();
      backRight.stop();
  }

  public void resetEncoders() {
      frontLeft.resetEncoders();
      frontRight.resetEncoders();
      backLeft.resetEncoders();
      backRight.resetEncoders();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
      frontLeft.setDesiredState(desiredStates[0]);
      frontRight.setDesiredState(desiredStates[1]);
      backLeft.setDesiredState(desiredStates[2]);
      backRight.setDesiredState(desiredStates[3]);
  }

}
