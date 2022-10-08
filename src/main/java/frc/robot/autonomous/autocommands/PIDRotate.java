// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.autocommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDRotate extends CommandBase {

  private final SwerveSubsystem SWERVE_SUBSYSTEM;
  private double goalAngle;
  int loopsInsideAllowableError;

  // Value used to scale the x-offset of the limelight when targeting
  // .1 was too fast, .01 was a little too slow, maybe try .02.
  PIDController thetaController;

  /** Creates a new GoalCentricCommand where the robot follows the goal rotationally as it translates in field-oriented mode. */
  public PIDRotate(SwerveSubsystem swerve, double goalAngle) {

    // Set instance variables equal to what was passed in
    this.SWERVE_SUBSYSTEM = swerve;

    this.goalAngle = swerve.getHeading(goalAngle);
    
    thetaController = new PIDController(.12, 0, 0.001);

    loopsInsideAllowableError = 0;
    // Require the swerve subsystem to allow for it to be a default command
    addRequirements(SWERVE_SUBSYSTEM);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopsInsideAllowableError = 0;
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


   // SmartDashboar.putNumber("Error Angle thing", goalAngle-SWERVE_SUBSYSTEM.getRawHeading());
    // Instead of turning based off an axis value, lets use the horizontal angle to the 
    // target measured by the limelight times some value to make it fit within the scope of robot control
    //double turningSpeed = (goalAngle - SWERVE_SUBSYSTEM.getHeading()) * kP;
    double turningSpeed = thetaController.calculate(SWERVE_SUBSYSTEM.getRawHeading(), goalAngle);

    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // Handle field-oriented driving with vision tracking input
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, SWERVE_SUBSYSTEM.getRotation2d());
    
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // FINALLY, set the output of the above calculation to each wheel
    SWERVE_SUBSYSTEM.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SWERVE_SUBSYSTEM.stop();
    
  }

 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(goalAngle - SWERVE_SUBSYSTEM.getRawHeading()) < 10) {
      loopsInsideAllowableError++;
    } else {
      loopsInsideAllowableError = 0;
    }

    // If we have been within the threshold for a certain number of loops, we deem that the setpoint has been reached, and the command exits.
    return loopsInsideAllowableError >= 10;
  }
}
