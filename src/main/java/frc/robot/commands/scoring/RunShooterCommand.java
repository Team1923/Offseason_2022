// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends CommandBase {

  private ShooterSubsystem SHOOTER_SUBSYSTEM;
  private ConveyorSubsystem CONVEYOR_SUBSYSTEM;
  private double goal_rpm;
  private Timer threshold_timer;
  private boolean canFire;

  /** Creates a new RunShooterRPM. */
  public RunShooterCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor, double rpm) {
    this.SHOOTER_SUBSYSTEM = shooter;
    this.CONVEYOR_SUBSYSTEM = conveyor;
    this.goal_rpm = rpm;

    threshold_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SHOOTER_SUBSYSTEM, CONVEYOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    threshold_timer.reset();
    threshold_timer.stop();

    canFire = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SHOOTER_SUBSYSTEM.setShooterWheelsRPM(-goal_rpm);

    // If we are within a certain bound of RPM from the RPM goal, start at timer. Otherwise, reset and stop the timer.
    if (Math.abs(Math.abs(SHOOTER_SUBSYSTEM.getShooterRPM())-Math.abs(goal_rpm)) < ShooterConstants.shooterRPMThreshold) {
      threshold_timer.start();
    } else {
      System.out.println("FUCK YOU");
      threshold_timer.reset();
      threshold_timer.stop();
    }

    // If the timer is above a certain value, we deem that the shooter is in an acceptable range.
    canFire = threshold_timer.get() > ShooterConstants.shooterTimeThreshold;
    //SmartDashboard.putNumber("THRESHOLD TIMER: ",threshold_timer.get());


    SHOOTER_SUBSYSTEM.setAcceptableRPMState(canFire);
    //SmartDashboard.putBoolean("Can Fire?", canFire);
    if(canFire) {
      CONVEYOR_SUBSYSTEM.setConveyor(ConveyorConstants.conveyorShootPercentOut);
    } else {
      CONVEYOR_SUBSYSTEM.stop();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.stop();
    CONVEYOR_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
