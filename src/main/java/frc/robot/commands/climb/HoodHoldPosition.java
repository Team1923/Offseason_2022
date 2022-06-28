// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import javax.swing.colorchooser.ColorSelectionModel;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateHandler;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.subsystems.HoodSubsystem;


// This command is useful for single-position goals for the hood. During teleop, we normally will have a
// constantly changing setpoint with time as we move closer and farther from the goal. However, during
// the climb sequence, we will have multiple occurances of wanting the climber to achieve a certain position
// and report back when it has done so. This command aims to achieve that goal.
public class HoodHoldPosition extends CommandBase {

  private HoodSubsystem HOOD_SUBSYSTEM;
  
  private double goal;

  private StateHandler stateHandler;
  private MKIPicoColorSensor colorSensor;


  /** Creates a new HoodSingleSetpointCommand. */
  public HoodHoldPosition(HoodSubsystem hood, double position, StateHandler state, MKIPicoColorSensor color) {
    this.HOOD_SUBSYSTEM = hood;
    this.goal = position;
    this.stateHandler = state;
    this.colorSensor = color;
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
    switch(stateHandler.getState()){
      case NO_BALLS:
        HOOD_SUBSYSTEM.setHoodPosition(goal);
        break;
      case ONE_BALL_CLOSE_BROKEN:
        HOOD_SUBSYSTEM.setHoodPosition(goal);
        break;
      case ONE_BALL_NONE_BROKEN:
        HOOD_SUBSYSTEM.setHoodPosition(goal);
        break;
      case ONE_BALL_FAR_BROKEN:
        if((colorSensor.isRed(1) && !colorSensor.isRedAndRed()) || (colorSensor.isBlue(1) && !colorSensor.isBlueAndBlue()))
          HOOD_SUBSYSTEM.setHoodPosition(40000);
        break;
      case TWO_BALLS_BOTH_BROKEN:
        HOOD_SUBSYSTEM.setHoodPosition(goal);
        break;
      default:
        HOOD_SUBSYSTEM.setHoodPosition(goal);
        break;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HOOD_SUBSYSTEM.setShootConstants();
    HOOD_SUBSYSTEM.stop();
    System.out.println("EXIT!!");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
