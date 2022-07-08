// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodApplyVoltage extends CommandBase {

  private HoodSubsystem HOOD_SUBSYSTEM;
  private double goal;

  /** Creates a new HoodApplyVoltage. */
  public HoodApplyVoltage(HoodSubsystem hoodSubsystem, double output) {

    this.HOOD_SUBSYSTEM = hoodSubsystem;
    this.goal = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(HOOD_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HOOD_SUBSYSTEM.set(goal);
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
