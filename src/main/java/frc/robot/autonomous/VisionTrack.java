// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.function.Supplier;

import org.opencv.core.Point;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionTrack extends CommandBase {

  private final SwerveSubsystem SWERVE_SUBSYSTEM;
  private final LimelightSubsystem LIMELIGHT_SUBSYSTEM;
  private final Supplier<Double> xSpdFunction, ySpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  // Value used to scale the x-offset of the limelight when targeting
  // .1 was too fast, .01 was a little too slow, maybe try .02.
  private final double kP = .015;

  /** Creates a new GoalCentricCommand where the robot follows the goal rotationally as it translates in field-oriented mode. */
  public VisionTrack(SwerveSubsystem swerve, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
                            LimelightSubsystem limelight) {

    // Set instance variables equal to what was passed in
    this.SWERVE_SUBSYSTEM = swerve;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.LIMELIGHT_SUBSYSTEM = limelight;
    
    // Slew Rate Limiters for smoother driving
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Require the swerve subsystem to allow for it to be a default command
    addRequirements(SWERVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LIMELIGHT_SUBSYSTEM.isGoalCentric = true;

    // Get real-time joystick values
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();

    // Instead of turning based off an axis value, lets use the horizontal angle to the 
    // target measured by the limelight times some value to make it fit within the scope of robot control
    double turningSpeed;
    if(LIMELIGHT_SUBSYSTEM.hasTarget()){
      turningSpeed = LIMELIGHT_SUBSYSTEM.getX() * kP;
    }
    else{
      turningSpeed = 0;
    }

    // Apply a deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // Make the driving smoother and make the max value the physical max speed of the robot
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // Handle field-oriented driving with vision tracking input
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, SWERVE_SUBSYSTEM.getRotation2d());
    
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // FINALLY, set the output of the above calculation to each wheel
    SWERVE_SUBSYSTEM.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SWERVE_SUBSYSTEM.stop();
    LIMELIGHT_SUBSYSTEM.isGoalCentric = false;
  }

  public double getHeadingTowardHub(){
    Point hub = Constants.hubPosition;
    double xHub = hub.x;
    double yHub = hub.y;
    double xCurrentRobot = SWERVE_SUBSYSTEM.getPose().getX();
    double yCurrentRobot = SWERVE_SUBSYSTEM.getPose().getY();
    double currentAngle = SWERVE_SUBSYSTEM.getPose().getRotation().getDegrees();
    if(xHub > xCurrentRobot){
      return (180 + currentAngle) - Math.toDegrees(Math.atan((xCurrentRobot - xHub) / (yCurrentRobot - yHub)));
    }
    else{
      return Math.toDegrees(Math.atan((xCurrentRobot - xHub) / (yCurrentRobot - yHub)));
    }

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
