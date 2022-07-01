// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateHandler;
import frc.robot.subsystems.HoodSubsystem;


// This command is useful for single-position goals for the hood. During teleop, we normally will have a
// constantly changing setpoint with time as we move closer and farther from the goal. However, during
// the climb sequence, we will have multiple occurances of wanting the climber to achieve a certain position
// and report back when it has done so. This command aims to achieve that goal.
public class DefaultHoodCommand extends CommandBase {

  private HoodSubsystem HOOD_SUBSYSTEM;
  private StateHandler stateHandler;


  /** Creates a new HoodSingleSetpointCommand. */
  public DefaultHoodCommand(HoodSubsystem hood, StateHandler state) {
    this.HOOD_SUBSYSTEM = hood;
    this.stateHandler = state;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(HOOD_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //configure PID for hood
    HOOD_SUBSYSTEM.setClimbConstants();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(stateHandler.getState()) {
      case TWO_BALLS_ONE_BROKEN:
        break;
      default:
        HOOD_SUBSYSTEM.setHoodPosition(0);
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HOOD_SUBSYSTEM.setShootConstants();
    HOOD_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
