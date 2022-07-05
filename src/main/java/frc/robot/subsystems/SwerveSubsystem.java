// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.interfaces.LimelightInterface;
import frc.robot.interfaces.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  // Instantiating four modules using the constants defined in the DriveConstants class in the constants file
  public final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveReversed,
      DriveConstants.kFrontLeftTurningReversed,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetReversed);

  public final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveReversed,
      DriveConstants.kFrontRightTurningReversed,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetReversed);

  public final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveReversed,
      DriveConstants.kBackRightTurningReversed,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetReversed);

  public final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveReversed,
      DriveConstants.kBackLeftTurningReversed,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetReversed);

  // ? Code is using a Pigeon 1 for testing, we will be upgrading to a Pigeon 2 so make sure that this gets updated to reflect that!
  private Pigeon2 gyro = new Pigeon2(Constants.kPigeonCANID, "Default Name");
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(LimelightInterface lInterface) {


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
    SmartDashboard.putNumber("Robot Heading: ", getRawHeading());
    //SmartDashboard.putString("Robot Location: ", getPose().getTranslation().toString());
    //SmartDashboard.putString("Robot Orientation: ", getPose().getRotation().toString());

    SmartDashboard.putNumber("Front Left", frontLeft.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("Front Right", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Left", backLeft.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("Back Right", backRight.getAbsoluteEncoderRad());

   // SmartDashboard.putNumber("Gyro Velocity Value: ", MKIMath.magnitude(getVelocityMagnitude()));

    odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());



    SmartDashboard.putString("Odometer Position", odometer.getPoseMeters().toString());

    //SmartDashboard.putNumber("Front Right Position", frontRight.getTurningPositionRads());
    //SmartDashboard.putNumber("Front Left Position", frontLeft.getTurningPositionRads());
    //SmartDashboard.putNumber("Back Right Position", backRight.getTurningPositionRads());
    //SmartDashboard.putNumber("Back Left Position", backLeft.getTurningPositionRads());



  }

  // Zeros the heading of the gyro
  public void zeroHeading() {
    gyro.setYaw(0);
  }

  public void setHeading(double heading) {
    gyro.setYaw(-heading);
  }

  // Gets a gyro heading between 0 and 360 degrees
  public double getHeading() {
    return -Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public double getRawHeading() {
    return -gyro.getYaw();
  }

  public void resetEncoders(){
    backLeft.resetEncoders();
    backRight.resetEncoders();
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
  }
  // Returns a Rotation2d object from the gyro heading, for use with swerve classes
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(pose, getRotation2d());
  }

  // Stop all of the modules
  public void stop() {
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
    backRight.setDesiredState(desiredStates[3]);
    backLeft.setDesiredState(desiredStates[2]);
  }

  public void resetStates() {
    frontLeft.resetState();
    frontRight.resetState();
    backRight.resetState();
    backLeft.resetState();
  }

  public List<Double> getVelocityMagnitude() {
   // gyro.getAccumGyro(gyroArray);
    //System.out.println(gyroArray);

    // return Arrays.asList(gyroArray[0] * Constants.gyroToMPS, gyroArray[1] * Constants.gyroToMPS);

    return Arrays.asList(1.0,1.0);
  }

}
