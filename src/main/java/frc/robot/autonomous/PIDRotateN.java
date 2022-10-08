// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDRotateN extends CommandBase {

  private final SwerveSubsystem swerve;
  private double goalAngle;
  int loopsInsideAllowableError;

  

  private boolean absolute;
  private double goal;

  // Value used to scale the x-offset of the limelight when targeting
  // .1 was too fast, .01 was a little too slow, maybe try .02.
  PIDController thetaController;

  // Boolean will det
  public PIDRotateN(SwerveSubsystem swerve, double goal, boolean absolute) {

    // Set instance variables equal to what was passed in
    this.swerve = swerve;
    this.goal = goal;
    this.absolute = absolute;

    
    
    thetaController = new PIDController(.24, 0, 0.015);
    thetaController.enableContinuousInput(0, 360);
    loopsInsideAllowableError = 0;
    // Require the swerve subsystem to allow for it to be a default command
    addRequirements(swerve);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("IN ROTATE?", false);
    loopsInsideAllowableError = 0;

    if(!absolute) {
      this.goalAngle = swerve.getHeading(goal);
    } else {
      this.goalAngle = goal;
    }
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("IN ROTATE?", true);


    SmartDashboard.putNumber("Error ", thetaController.getPositionError());
    // Instead of turning based off an axis value, lets use the horizontal angle to the 
    // target measured by the limelight times some value to make it fit within the scope of robot control
    //double turningSpeed = (goalAngle - SWERVE_SUBSYSTEM.getHeading()) * kP;
    double turningSpeed = thetaController.calculate(swerve.getHeading(), goalAngle);
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // Handle field-oriented driving with vision tracking input
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerve.getRotation2d());
    
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // FINALLY, set the output of the above calculation to each wheel
    swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("IN ROTATE?", false);
    swerve.stop();
    
  }

 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //goalAngle - swerve.getHeading()
    if(Math.abs(thetaController.getPositionError()) < 5) {
      loopsInsideAllowableError++;
    } else {
      loopsInsideAllowableError = 0;
    }

    // If we have been within the threshold for a certain number of loops, we deem that the setpoint has been reached, and the command exits.
    return loopsInsideAllowableError >= 10;
  }
}
