// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAvoidStallCommand extends CommandBase {

  private ShooterSubsystem SHOOTER_SUBSYSTEM;

  /** Creates a new ShooterAvoidStallCommand. */
  public ShooterAvoidStallCommand(ShooterSubsystem shooter) {
    this.SHOOTER_SUBSYSTEM = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SHOOTER_SUBSYSTEM.set(-ShooterConstants.avoidStallSpeed);
    //SmartDashboard.putNumber("SYSTEM TIME SHOOTER: ", System.currentTimeMillis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
