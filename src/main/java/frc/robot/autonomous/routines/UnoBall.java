// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.autocommands.AutoShoot;
import frc.robot.autonomous.autocommands.RunTrajectory;
import frc.robot.commands.climb.HoodSingleSetpointCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.StateHandler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnoBall extends SequentialCommandGroup {
  /** Creates a new UnoBall. */
  public UnoBall(SwerveSubsystem swerve, ShooterSubsystem shooter, ConveyorSubsystem conveyor, IntakeSubsystem intake, HoodSubsystem hood, StateHandler stateHandler) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      /*
        Things to TEST:
          - Is the shot good? (shooter was re-tuned, not sure if this is a viable fender shot)
          - Does the oneBallPath cross the tarmac line?
    */
      new HoodSingleSetpointCommand(hood, 0),
      new AutoShoot(shooter, conveyor, hood, 0, 2750).withTimeout(1.5),
      new RunTrajectory(swerve, "oneBallPath", true),
      new InstantCommand(() -> stateHandler.resetState())
    );
  }
}
