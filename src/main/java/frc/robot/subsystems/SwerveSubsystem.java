// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {


  // Instantiating four modules using the constants defined in the DriveConstants class in the constants file
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveReversed,
      DriveConstants.kFrontLeftTurningReversed,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetReversed);

  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveReversed,
      DriveConstants.kFrontRightTurningReversed,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetReversed);

  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveReversed,
      DriveConstants.kBackRightTurningReversed,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetReversed);

  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveReversed,
      DriveConstants.kBackLeftTurningReversed,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetReversed);

  // ? Code is using a Pigeon 1 for testing, we will be upgrading to a Pigeon 2 so make sure that this gets updated to reflect that!
  private PigeonIMU gyro = new PigeonIMU(Constants.kPigeonCANID);

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    // Gyro is going to be calibrating for first part of boot, so delay wipe of settings and zeroing of heading by a second each on a seperate thread
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro.configFactoryDefault();
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    }).start();

  }

  @Override
  public void periodic() {
    // Prints out robot heading for debug purposes
    SmartDashboard.putNumber("Robot Heading: ", getHeading());
  }

  // Zeros the heading of the gyro
  public void zeroHeading() {
    gyro.setFusedHeading(0);
    gyro.setAccumZAngle(0);
  }

  // Gets a gyro heading between 0 and 360 degrees
  public double getHeading() {
    return Math.IEEEremainder(gyro.getFusedHeading(), 360);
  }

  // Returns a Rotation2d object from the gyro heading, for use with swerve classes
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  // Stop all of the modules
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  // Update the modules states from an array of states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    
    // Normalize the wheel speeds based on the physical max speed of the robot
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
    // Actually send the desired states to the respective swerve modules
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backRight.setDesiredState(desiredStates[2]);
    frontLeft.setDesiredState(desiredStates[3]);
  }

}
