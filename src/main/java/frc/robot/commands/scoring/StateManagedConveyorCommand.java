// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateHandler;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.StateHandler.States;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StateManagedConveyorCommand extends CommandBase {

  private ConveyorSubsystem CONVEYOR_SUBSYSTEM;
  private StateHandler stateHandler;
  private States currentRobotState;
  private MKIPicoColorSensor colorSensor;
  private ShooterSubsystem shooterSubsystem;
  private Timer shittyTimer;
  /** 
   *  Defines a new 
   * 
   * @param intake The singleton intake subsystem.
   * @param handler The singleton state handler.
   * @param intaking A boolean defining whether the robot is currently intaking vs shooting.
   */
  public StateManagedConveyorCommand(ConveyorSubsystem conveyor, ShooterSubsystem shooter, StateHandler handler, MKIPicoColorSensor color) {
    this.CONVEYOR_SUBSYSTEM = conveyor;
    this.stateHandler = handler;
    this.colorSensor = color;
    this.shooterSubsystem = shooter;
    shittyTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CONVEYOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Update the robot state in this command with the actual current robot state from the state handler.
    currentRobotState = stateHandler.getState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update the robot state in this command with the actual current robot state from the state handler.
    currentRobotState = stateHandler.getState();

    double conveyorPercentOut = stateHandler.getEjecting() ?  ConveyorConstants.conveyorOutPercentOut : ConveyorConstants.conveyorPercentOut;
      switch(currentRobotState) {
        case NO_BALLS:
          CONVEYOR_SUBSYSTEM.stop();
          break;
        case ONE_BALL_CLOSE_BROKEN:
          CONVEYOR_SUBSYSTEM.setConveyor(conveyorPercentOut);
          break;
        case ONE_BALL_FAR_BROKEN:
          //if((colorSensor.isRed(1) && !colorSensor.isRedAndRed()) || (colorSensor.isBlue(1) && !colorSensor.isBlueAndBlue())){
            if(shooterSubsystem.getAcceptableRPMState()){
              CONVEYOR_SUBSYSTEM.setConveyor(ConveyorConstants.conveyorShootPercentOut);
              System.out.println("SUCK");
            }
          //}
          else{
            System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            CONVEYOR_SUBSYSTEM.stop();
          }
          
          break;
        case ONE_BALL_NONE_BROKEN:
          
          CONVEYOR_SUBSYSTEM.stop();
          break;
        case TWO_BALLS_BOTH_BROKEN:
          // if((colorSensor.isRed(1) && !colorSensor.isRedAndRed()) || (colorSensor.isBlue(1) && !colorSensor.isBlueAndBlue())){
            if(shooterSubsystem.getAcceptableRPMState()){
              CONVEYOR_SUBSYSTEM.setConveyor(ConveyorConstants.conveyorShootPercentOut);
            }
          //}
          else{
            CONVEYOR_SUBSYSTEM.stop();
          }
          break;
        default:
          CONVEYOR_SUBSYSTEM.stop();
          break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYOR_SUBSYSTEM.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
