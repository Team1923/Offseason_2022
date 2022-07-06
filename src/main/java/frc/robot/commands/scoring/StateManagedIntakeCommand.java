// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateHandler;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StateManagedIntakeCommand extends CommandBase {
  private IntakeSubsystem intake;
  private StateHandler stateHandler;
  private boolean intakeReversed;
  private ConveyorSubsystem conveyor;

  public StateManagedIntakeCommand(IntakeSubsystem intake, StateHandler stateHandler, ConveyorSubsystem conveyor, boolean reversed, MKIPicoColorSensor colorSensor) {
    this.intake = intake;
    this.stateHandler = stateHandler;
    this.conveyor = conveyor;
    intakeReversed = reversed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intake_reverse = intakeReversed;
    intake.setHigh();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakePercentOut;
    if(intakeReversed){
      intakePercentOut = IntakeConstants.intakeOutPercentOut;
    }
    else{
      intakePercentOut = IntakeConstants.intakePercentOut;
    }

      if(stateHandler.getEjecting()){ 
        switch(stateHandler.getState()){
          case NO_BALLS:
            intake.setIntake(intakePercentOut);
            conveyor.setConveyor(ConveyorConstants.conveyorOutPercentOut);
            break;
          case ONE_BALL_CLOSE_BROKEN:
            intake.setIntake(intakePercentOut);
            conveyor.setConveyor(ConveyorConstants.conveyorOutPercentOut);
            break;
          case ONE_BALL_NONE_BROKEN:
            intake.setIntake(intakePercentOut);
            conveyor.setConveyor(ConveyorConstants.conveyorOutPercentOut);
            break;
          case ONE_BALL_FAR_BROKEN:
            intake.setIntake(intakePercentOut);
            conveyor.setConveyor(ConveyorConstants.conveyorOutPercentOut);
            break;
          case TWO_BALLS_BOTH_BROKEN:
            intake.setIntake(intakePercentOut);
            conveyor.setConveyor(ConveyorConstants.conveyorOutPercentOut);
            break;
          default:
            intake.stop();
            break;
        }

      } else {
        
        switch(stateHandler.getState()){
          case NO_BALLS:
            intake.setIntake(intakePercentOut);
            break;
          case ONE_BALL_CLOSE_BROKEN:
            intake.setIntake(intakePercentOut);
            break;
          case ONE_BALL_NONE_BROKEN:
            intake.setIntake(intakePercentOut);
            break;
          case ONE_BALL_FAR_BROKEN:
            intake.setIntake(intakePercentOut);
            break;
          case TWO_BALLS_BOTH_BROKEN:
            intake.stop();
            break;
          default:
            intake.stop();
            break;
        }

      }
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.setLow();
    conveyor.stop();
    intake.intake_reverse = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
