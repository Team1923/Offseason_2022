// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends CommandBase {

  private ShooterSubsystem SHOOTER_SUBSYSTEM;
  private double goal_rpm;
  private Timer threshold_timer;

  /** Creates a new RunShooterRPM. */
  public RunShooterCommand(ShooterSubsystem shooter, double rpm) {
    this.SHOOTER_SUBSYSTEM = shooter;
    this.goal_rpm = rpm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SHOOTER_SUBSYSTEM.setShooterWheelsRPM(goal_rpm);

    threshold_timer.reset();
    threshold_timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If we are within a certain bound of RPM from the RPM goal, start at timer. Otherwise, reset and stop the timer.
    if (Math.abs(SHOOTER_SUBSYSTEM.getShooterRPM()-goal_rpm) < ShooterConstants.shooterRPMThreshold) {
      threshold_timer.start();
    } else {
      threshold_timer.reset();
      threshold_timer.stop();
    }

    // If the timer is above a certain value, we deem that the shooter is in an acceptable range.
    if (threshold_timer.get() > ShooterConstants.shooterTimeThreshold) {
      SHOOTER_SUBSYSTEM.setAcceptableRPMState(true);
    } else {
      SHOOTER_SUBSYSTEM.setAcceptableRPMState(false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
