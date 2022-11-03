// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetReversed,
      false);

  public final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveReversed,
      DriveConstants.kFrontRightTurningReversed,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetReversed,
      false);

  public final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveReversed,
      DriveConstants.kBackRightTurningReversed,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetReversed,
      true);

  public final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveReversed,
      DriveConstants.kBackLeftTurningReversed,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetReversed,
      true);

  // ? Code is using a Pigeon 1 for testing, we will be upgrading to a Pigeon 2 so make sure that this gets updated to reflect that!
  private Pigeon2 gyro = new Pigeon2(Constants.kPigeonCANID, "Default Name");
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d());
  private final SwerveDriveOdometry auto_odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d());

  private double keepAngle = 0.0; // Double to store the current target keepAngle in radians
  private double timeSinceRot = 0.0; // Double to store the time since last rotation command
  private double lastRotTime = 0.0; // Double to store the time of the last rotation command
  private double timeSinceDrive = 0.0; // Double to store the time since last translation command
  private double lastDriveTime = 0.0; // Double to store the time of the last translation command

  private final Timer keepAngleTimer = new Timer(); // Creates timer used in the perform keep angle function

  private final PIDController m_keepAnglePID = new PIDController(.5,0,0);

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

  //  SmartDashboard.putNumber("ACTUAL Front Left Turning Encoder", frontLeft.getTurningTicks());
  //  SmartDashboard.putNumber("ACTUAL Front Right Turning Encoder", frontRight.getTurningTicks());
  //  SmartDashboard.putNumber("ACTUAL Back Left Turning Encoder", backLeft.getTurningTicks());
  //  SmartDashboard.putNumber("ACTUAL Back Right Turning Encoder", backRight.getTurningTicks());
    
   SmartDashboard.putNumber("0Front Left", frontLeft.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("0Front Right", frontRight.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("0Back Left", backLeft.getAbsoluteEncoderRad());
   SmartDashboard.putNumber("0Back Right", backRight.getAbsoluteEncoderRad());
  
    SmartDashboard.putNumber("Auto Odom X: ", auto_odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("Auto Odom Y: ", auto_odometer.getPoseMeters().getY());
    SmartDashboard.putNumber("Auto Odom Theta: ", auto_odometer.getPoseMeters().getRotation().getDegrees());
    //SmartDashboard.putNumber("PID ERROR", frontLeft.getPIDError());
    
    odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());

   // SmartDashboar.putString("Odometer Position", odometer.getPoseMeters().toString());

    SmartDashboard.putNumber("Heading", getHeading());
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
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public double getHeading(double offset) {
    return -Math.IEEEremainder(gyro.getYaw() + offset, 360);
  }
  
  public double getRawHeading() {
    return -gyro.getYaw();
  }

  public double getPitch(){
    return -Math.IEEEremainder(gyro.getPitch(), 360);
  }

  // Returns a Rotation2d object from the gyro heading, for use with swerve classes
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public Pose2d getAutoPose() {
    auto_odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    
    return auto_odometer.getPoseMeters();
  }
  

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(pose, getRotation2d());
    auto_odometer.resetPosition(pose, getRotation2d());
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
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void resetStates() {
    frontLeft.resetState();
    frontRight.resetState();
    backRight.resetState();
    backLeft.resetState();
  }

  public ChassisSpeeds getChassisSpeed() {

    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(), 
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  // PID controller to avoid rotational drifting that we experience during a match. From 1706's Github. Thanks for the help
  public double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
                         // called
    if (Math.abs(rot) >= DriveConstants.kMinRotationCommand) { // If the driver commands the robot to rotate set the
                                                               // last rotate time to the current time
      lastRotTime = keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommand
        || Math.abs(ySpeed) >= DriveConstants.kMinTranslationCommand) { // if driver commands robot to translate set the
                                                                        // last drive time to the current time
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time
    timeSinceDrive = keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive time
    if (timeSinceRot < 0.5) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
                              // move to finish
      keepAngle = getRotation2d().getRadians();
    } else if (Math.abs(rot) < DriveConstants.kMinRotationCommand && timeSinceDrive < 0.25) { // Run Keep angle pid
                                                                                              // until 0.75s after drive
                                                                                              // command stops to combat
                                                                                              // decel drift
      output = m_keepAnglePID.calculate(getRotation2d().getRadians(), keepAngle); // Set output command to the result of the
                                                                            // Keep Angle PID
    }
    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getRotation2d().getRadians();
  }

  public void turningPercentOutput(){
    frontLeft.turningMotorPercentOut();
    frontRight.turningMotorPercentOut();
    backLeft.turningMotorPercentOut();
    backRight.turningMotorPercentOut();

  }

  
}
