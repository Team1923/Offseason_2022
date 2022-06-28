// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbApplyVoltage extends CommandBase {

  ClimbSubsystem CLIMB_SUBSYSTEM;
  double output;

  /** Creates a new ClimbApplyVoltage. */
  public ClimbApplyVoltage(ClimbSubsystem climb, double goal) {
    this.CLIMB_SUBSYSTEM = climb;
    this.output = -goal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CLIMB_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CLIMB_SUBSYSTEM.set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CLIMB_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
