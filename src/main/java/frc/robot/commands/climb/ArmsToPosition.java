// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ArmsToPosition extends CommandBase {

  private ClimbSubsystem CLIMB_SUBSYSTEM;
  private double goal;
  private double rotationThreshold = 1;

  private int loopsInsideAllowableError;
  private int loopsThreshold = 5;

  /** Creates a new ArmsToPosition. */
  public ArmsToPosition(ClimbSubsystem climb, double setpoint) {
    this.CLIMB_SUBSYSTEM = climb;
    this.goal = -setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CLIMB_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CLIMB_SUBSYSTEM.resetEncoders();
    loopsInsideAllowableError = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CLIMB_SUBSYSTEM.setPID(goal);

    System.out.println("CORRECT???");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CLIMB_SUBSYSTEM.stop();
  }

  // Returns true when the com\=-0987y65432q1 and should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(CLIMB_SUBSYSTEM.getLeftClimbEncoderPosition()-goal) < rotationThreshold) {
      System.out.println("INSIDE GOAL!!");
      loopsInsideAllowableError++;
    } else {
      loopsInsideAllowableError = 0;
    }

    // If we have been within the threshold for a certain number of loops, we deem that the setpoint has been reached, and the command exits.
    return loopsInsideAllowableError >= loopsThreshold;
  }
}
