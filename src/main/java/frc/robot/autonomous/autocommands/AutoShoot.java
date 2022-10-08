// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends CommandBase {
  private ShooterSubsystem shooter;
  private ConveyorSubsystem conveyor;
  private HoodSubsystem hood;
  private double hoodSetpoint, shooterVel;
  private Timer threshold_timer;
  private boolean canFire;
  int loopsInsideAllowableError = 0;
  boolean hoodGood;
  public AutoShoot(ShooterSubsystem shooter, ConveyorSubsystem conveyor, HoodSubsystem hood, double stpt, double vel) {
      this.shooter = shooter;
      this.conveyor = conveyor;
      this.hood = hood;
      this.hoodSetpoint = stpt;
      this.shooterVel = vel;
      threshold_timer = new Timer();
      canFire = false;
      hoodGood = false;
      addRequirements(shooter, conveyor, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setHoodPosition(hoodSetpoint);
    canFire = false;
    loopsInsideAllowableError = 0;
    hoodGood = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterWheelsRPM(-shooterVel);
    hood.setHoodPosition(hoodSetpoint);
    if (Math.abs(Math.abs(shooter.getShooterRPM())-Math.abs(shooterVel)) < ShooterConstants.shooterRPMThreshold) {
      threshold_timer.start();
    } else {
      threshold_timer.reset();
      threshold_timer.stop();
    }

    if(Math.abs(hood.getPosition()-hoodSetpoint) < 200) {
      loopsInsideAllowableError++;
    } else {
      loopsInsideAllowableError = 0;
    }

    // If we have been within the threshold for a certain number of loops, we deem that the setpoint has been reached, and the command exits.
    hoodGood = loopsInsideAllowableError >= 5;
    
    // If the timer is above a certain value, we deem that the shooter is in an acceptable range.
    canFire = threshold_timer.get() > ShooterConstants.shooterTimeThreshold;

    if(canFire && hoodGood){
      conveyor.setConveyor(ConveyorConstants.conveyorPercentOut);
    }
    else{
      conveyor.stop();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.stop();
    conveyor.stop();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
