// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorCommand extends CommandBase {

  private ConveyorSubsystem CONVEYOR_SUBSYSTEM;
  private boolean reverse_intake;

  /** Creates a new RunConveyorCommand. */
  public RunConveyorCommand(ConveyorSubsystem conveyor, boolean reverse) {
    this.CONVEYOR_SUBSYSTEM = conveyor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(CONVEYOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CONVEYOR_SUBSYSTEM.setConveyor(reverse_intake ? -ConveyorConstants.conveyorPercentOut : ConveyorConstants.conveyorPercentOut);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYOR_SUBSYSTEM.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
