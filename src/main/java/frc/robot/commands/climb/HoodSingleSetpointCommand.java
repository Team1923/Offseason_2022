// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;


// This command is useful for single-position goals for the hood. During teleop, we normally will have a
// constantly changing setpoint with time as we move closer and farther from the goal. However, during
// the climb sequence, we will have multiple occurances of wanting the climber to achieve a certain position
// and report back when it has done so. This command aims to achieve that goal.
public class HoodSingleSetpointCommand extends CommandBase {

  private HoodSubsystem HOOD_SUBSYSTEM;
  
  private double goal;

  private double loopsInsideAllowableError;
  private double loopsThreshold = 5;

  private double tickThreshold = 400;

  /** Creates a new HoodSingleSetpointCommand. */
  public HoodSingleSetpointCommand(HoodSubsystem hood, double position) {
    this.HOOD_SUBSYSTEM = hood;
    this.goal = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(HOOD_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //configure PID for hood
    HOOD_SUBSYSTEM.setClimbConstants();

    loopsInsideAllowableError = 0;

    HOOD_SUBSYSTEM.setHoodPosition(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HOOD_SUBSYSTEM.setShootConstants();
    HOOD_SUBSYSTEM.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This will check if the ERROR of the Motion Magic is within our acceptable error threshold (ticks).;If so, add one to the counter of how many loops we have been 
    // within this threshold. Otherwise, reset the counter.
    if(Math.abs(HOOD_SUBSYSTEM.getPosition()-goal) < tickThreshold) {
      loopsInsideAllowableError++;
    } else {
      loopsInsideAllowableError = 0;
    }

    // If we have been within the threshold for a certain number of loops, we deem that the setpoint has been reached, and the command exits.
    return loopsInsideAllowableError >= loopsThreshold;
  }
}
