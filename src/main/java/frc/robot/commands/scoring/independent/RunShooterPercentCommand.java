// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring.independent;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterPercentCommand extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private ConveyorSubsystem conveyorSubsystem;

  public RunShooterPercentCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
    shooterSubsystem = shooter;
    conveyorSubsystem = conveyor;

    addRequirements(shooterSubsystem, conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.set(-ShooterConstants.shooterPercentOut);
    conveyorSubsystem.setConveyor(ConveyorConstants.conveyorPercentOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    conveyorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
