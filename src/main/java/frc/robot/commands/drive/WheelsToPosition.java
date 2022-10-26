// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class WheelsToPosition extends CommandBase {

  private SwerveSubsystem swerve;
  private SwerveModuleState[] states = {
    new SwerveModuleState(1, new Rotation2d(Math.PI/4)),
    new SwerveModuleState(1, new Rotation2d(Math.PI/4)),
    new SwerveModuleState(1, new Rotation2d(Math.PI/4)),
    new SwerveModuleState(1, new Rotation2d(Math.PI/4))
  };

  /** Creates a new WheelsToPosition. */
  public WheelsToPosition(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("TEST");
    swerve.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
