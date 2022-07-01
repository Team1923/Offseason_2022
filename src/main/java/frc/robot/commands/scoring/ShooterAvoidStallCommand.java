// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateHandler;
import frc.robot.Constants.ShooterConstants;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAvoidStallCommand extends CommandBase {

  private ShooterSubsystem SHOOTER_SUBSYSTEM;
  private StateHandler stateHandler;
  private Timer threshold_timer;

  /** Creates a new ShooterAvoidStallCommand. */
  public ShooterAvoidStallCommand(ShooterSubsystem shooter, StateHandler state, MKIPicoColorSensor color) {
    this.SHOOTER_SUBSYSTEM = shooter;
    this.stateHandler = state;
    threshold_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(stateHandler.getState()){
      case NO_BALLS:
        SHOOTER_SUBSYSTEM.set(-ShooterConstants.avoidStallSpeed); 
        break;
      case ONE_BALL_CLOSE_BROKEN:
        SHOOTER_SUBSYSTEM.set(-ShooterConstants.avoidStallSpeed);
        break;
      case ONE_BALL_NONE_BROKEN:
        SHOOTER_SUBSYSTEM.set(-ShooterConstants.avoidStallSpeed);
        break;
      case ONE_BALL_FAR_BROKEN:

        SHOOTER_SUBSYSTEM.setShooterWheelsRPM(5000);

        if (Math.abs(Math.abs(SHOOTER_SUBSYSTEM.getShooterRPM())-Math.abs(5000)) < ShooterConstants.shooterRPMThreshold) {

          threshold_timer.start();

        } else {

          threshold_timer.reset();
          threshold_timer.stop();

        }

        SHOOTER_SUBSYSTEM.setAcceptableRPMState(threshold_timer.get() > ShooterConstants.shooterTimeThreshold);

        break;
      case TWO_BALLS_BOTH_BROKEN:
        SHOOTER_SUBSYSTEM.set(-ShooterConstants.avoidStallSpeed);
        break;
      default:
        SHOOTER_SUBSYSTEM.set(-ShooterConstants.avoidStallSpeed); 
        break;
    }
    
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
