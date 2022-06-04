// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroStates extends CommandBase {

  SwerveSubsystem SWERVE_SUBSYSTEM;

  /** Creates a new ResetWheelCommand. */
  public ZeroStates(SwerveSubsystem swerve) {
    this.SWERVE_SUBSYSTEM = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SWERVE_SUBSYSTEM.resetStates();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SWERVE_SUBSYSTEM.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
