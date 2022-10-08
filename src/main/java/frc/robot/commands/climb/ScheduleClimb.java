// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.DesiredClimb;

public class ScheduleClimb extends CommandBase {

  DesiredClimb desiredClimb;
  HoodSubsystem HOOD_SUBSYSTEM;
  ClimbSubsystem CLIMB_SUBSYSTEM;
  Supplier<Boolean> commit;
  ConveyorSubsystem conveyor;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  SwerveSubsystem swerve;

  /** Creates a new ScheduleClimb. */
  public ScheduleClimb(DesiredClimb dc, HoodSubsystem hood, ClimbSubsystem climb, Supplier<Boolean> supplier, ConveyorSubsystem conveyor, IntakeSubsystem intake, ShooterSubsystem shooter, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencie
    this.desiredClimb = dc;
    this.HOOD_SUBSYSTEM = hood;
    this.CLIMB_SUBSYSTEM = climb;
    this.commit = supplier;
    this.conveyor = conveyor;
    this.intake = intake;
    this.shooter = shooter;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // SmartDashboar.putString("CURRENT CLIMB PRE GO: ", desiredClimb.getCurrentClimb().toString());

    switch(desiredClimb.getCurrentClimb()) {
      case FULL_TRAVERSAL:
       // SmartDashboar.putNumber("CLIMB VALUE: ", 1);
        CommandScheduler.getInstance().schedule(new FullTraversalClimbSequence(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit, conveyor, shooter, swerve, intake));
        break;
      case LEVEL_THREE:
     // SmartDashboar.putNumber("CLIMB VALUE: ", 2);
        CommandScheduler.getInstance().schedule(new LevelThreeClimb(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit));
        break;
      case LEVEL_TWO:
     // SmartDashboar.putNumber("CLIMB VALUE: ", 3);

        CommandScheduler.getInstance().schedule(new LevelTwoClimb(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit));
        break;
      case TRAVERSAL_ARMS_EXTENDED:
     // SmartDashboar.putNumber("CLIMB VALUE: ", 4);

        CommandScheduler.getInstance().schedule(new TraversalArmsExtended(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit));
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
