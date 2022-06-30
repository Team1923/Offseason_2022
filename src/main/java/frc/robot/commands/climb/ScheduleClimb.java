// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.DesiredClimb;
import frc.robot.StateHandler;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class ScheduleClimb extends CommandBase {

  DesiredClimb desiredClimb;
  HoodSubsystem HOOD_SUBSYSTEM;
  ClimbSubsystem CLIMB_SUBSYSTEM;
  MKIPicoColorSensor colorSensor;
  StateHandler stateHandler;
  Supplier<Boolean> commit;

  /** Creates a new ScheduleClimb. */
  public ScheduleClimb(DesiredClimb dc, HoodSubsystem hood, ClimbSubsystem climb, MKIPicoColorSensor color, StateHandler state, Supplier<Boolean> supplier) {
    // Use addRequirements() here to declare subsystem dependencie
    this.desiredClimb = dc;
    this.HOOD_SUBSYSTEM = hood;
    this.CLIMB_SUBSYSTEM = climb;
    this.colorSensor = color;
    this.stateHandler = state;
    this.commit = supplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("CURRENT CLIMB PRE GO: ", desiredClimb.getCurrentClimb().toString());

    switch(desiredClimb.getCurrentClimb()) {
      case FULL_TRAVERSAL:
        SmartDashboard.putNumber("CLIMB VALUE: ", 1);
        CommandScheduler.getInstance().schedule(new FullTraversalClimbSequence(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit, stateHandler, colorSensor));
        break;
      case LEVEL_THREE:
      SmartDashboard.putNumber("CLIMB VALUE: ", 2);
        CommandScheduler.getInstance().schedule(new LevelThreeClimb(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit, stateHandler, colorSensor));
        break;
      case LEVEL_TWO:
      SmartDashboard.putNumber("CLIMB VALUE: ", 3);

        CommandScheduler.getInstance().schedule(new LevelTwoClimb(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit, stateHandler, colorSensor));
        break;
      case TRAVERSAL_ARMS_EXTENDED:
      SmartDashboard.putNumber("CLIMB VALUE: ", 4);

        CommandScheduler.getInstance().schedule(new TraversalArmsExtended(HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, commit, stateHandler, colorSensor));
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
