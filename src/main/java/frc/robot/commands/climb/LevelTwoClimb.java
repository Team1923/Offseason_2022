// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LevelTwoClimb extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  public LevelTwoClimb(HoodSubsystem HOOD_SUBSYSTEM, ClimbSubsystem CLIMB_SUBSYSTEM, Supplier<Boolean> commit) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeployArms(CLIMB_SUBSYSTEM).withTimeout(.5),
      new InstantCommand(() -> CLIMB_SUBSYSTEM.resetEncoders()),
      new ArmsToPosition(CLIMB_SUBSYSTEM, ClimbConstants.maxArmPositionTicks),
      new WaitForButton(commit),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new HoodSingleSetpointCommand(HOOD_SUBSYSTEM, 40100),
          new HoodApplyVoltage(HOOD_SUBSYSTEM, .1)
        ),
        new ArmsToPosition(CLIMB_SUBSYSTEM, 0)
      ),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
              new ClimbApplyVoltage(CLIMB_SUBSYSTEM, .3).withTimeout(1),
              new ArmsToPosition(CLIMB_SUBSYSTEM, 10)
            ),
            new HoodHoldPosition(HOOD_SUBSYSTEM, 36000)
        )
        
      
      
      // new ArmsToPosition(CLIMB_SUBSYSTEM, -20),
      // new HoodSingleSetpointCommand(HOOD_SUBSYSTEM, 25000),
      // new ArmsToPosition(CLIMB_SUBSYSTEM, -60)
    );
  }
}
