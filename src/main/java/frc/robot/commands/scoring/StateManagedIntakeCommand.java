// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateHandler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateHandler.States;
import frc.robot.subsystems.IntakeSubsystem;

public class StateManagedIntakeCommand extends CommandBase {
  private IntakeSubsystem intake;
  private StateHandler stateHandler;
  private States currentRobotState;

  public StateManagedIntakeCommand(IntakeSubsystem intake, StateHandler stateHandler) {
    this.intake = intake;
    this.stateHandler = stateHandler;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setHigh();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakePercentOut = stateHandler.getEjecting() ? -IntakeConstants.intakePercentOut : IntakeConstants.intakePercentOut;

      if(!stateHandler.getEjecting()){ //if the intake in in reverse
        switch(currentRobotState){
          case NO_BALLS:
            intake.stop();
          case ONE_BALL_CLOSE_BROKEN:
            intake.setIntake(intakePercentOut);
          case ONE_BALL_NONE_BROKEN:
            intake.setIntake(intakePercentOut);
          case ONE_BALL_FAR_BROKEN:
            intake.setIntake(intakePercentOut);
          case TWO_BALLS_BOTH_BROKEN:
            intake.setIntake(intakePercentOut);
        }
      }

      if(stateHandler.getEjecting()){
        switch(currentRobotState){
          case NO_BALLS:
            intake.setIntake(intakePercentOut);
          case ONE_BALL_CLOSE_BROKEN:
            intake.setIntake(intakePercentOut);
          case ONE_BALL_NONE_BROKEN:
            intake.setIntake(intakePercentOut);
          case ONE_BALL_FAR_BROKEN:
            intake.setIntake(intakePercentOut);
          case TWO_BALLS_BOTH_BROKEN:
            intake.stop();
        }
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.setLow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
