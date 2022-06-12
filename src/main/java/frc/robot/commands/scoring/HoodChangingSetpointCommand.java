// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class HoodChangingSetpointCommand extends CommandBase {

  private HoodSubsystem HOOD_SUBSYSTEM;
  private LimelightSubsystem LIMELIGHT_SUBSYSTEM;

  /** Creates a new HoodChangingSetpointCommand. */
  public HoodChangingSetpointCommand(HoodSubsystem hood, LimelightSubsystem limelight) {
    this.HOOD_SUBSYSTEM = hood;
    this.LIMELIGHT_SUBSYSTEM = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(HOOD_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HOOD_SUBSYSTEM.setShootConstants();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Map the angle between the camera and the goal (range of [0,90]) onto the range of
    // motion of the hood (range [minPosition, maxPosition]). 
    double tickGoal = LIMELIGHT_SUBSYSTEM.map(
        0, 
        90, 
        HoodConstants.hoodMinPosition, 
        HoodConstants.hoodMaxPosition, 
        LIMELIGHT_SUBSYSTEM.angle());

    // Adjust the generated angle by a scale factor. If the new angle is greater than the max
    // allowed angle, then cap it to the max allowed angle.
    if (tickGoal * HoodConstants.hoodScaleFactor > HoodConstants.hoodMaxPosition) {
      tickGoal = HoodConstants.hoodMaxPosition;
    } else {
      tickGoal *= HoodConstants.hoodScaleFactor;
    }

    HOOD_SUBSYSTEM.setHoodPosition(tickGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HOOD_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
