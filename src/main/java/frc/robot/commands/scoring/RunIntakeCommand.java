// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends CommandBase {

  private IntakeSubsystem INTAKE_SUBSYSTEM;
  private boolean reverse_intake;

  /** Creates a new ExtendIntakeCommand. */
  public RunIntakeCommand(IntakeSubsystem intake, boolean reverse) {
    this.INTAKE_SUBSYSTEM = intake;
    this.reverse_intake = reverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    INTAKE_SUBSYSTEM.setHigh();
    INTAKE_SUBSYSTEM.setIntake(reverse_intake ? -IntakeConstants.intakePercentOut : IntakeConstants.intakePercentOut);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.setLow();
    INTAKE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
