// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodResetHold extends CommandBase {

  HoodSubsystem HOOD_SUBSYSTEM;

  /** Creates a new HoodResetHold. */
  public HoodResetHold(HoodSubsystem hood) {
    this.HOOD_SUBSYSTEM = hood;
    addRequirements(HOOD_SUBSYSTEM);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HOOD_SUBSYSTEM.set(-.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HOOD_SUBSYSTEM.stop();
    HOOD_SUBSYSTEM.resetEncoder(); // CHANGE THIS TO THE NEW "HOOD OFFSET ZERO METHOD"
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
