// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.autocommands.AutoShoot;
import frc.robot.autonomous.autocommands.PIDRotate;
import frc.robot.autonomous.autocommands.RunTrajectory;
import frc.robot.commands.scoring.independent.RunIntakeCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallGetOut extends SequentialCommandGroup {
  /** Creates a new UnoBall. */
  public OneBallGetOut(SwerveSubsystem swerve, ShooterSubsystem shooter, ConveyorSubsystem conveyor, IntakeSubsystem intake, HoodSubsystem hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    /*
      Things to test/notes:
        - OneBallGetOut should only shoot one ball and acquire an enemy ball. The enemy ball should be randomly shot out. 
        - Path looks fine, but the final waypoint will need adjusting based on new measurements. Angle will also need adjusting. 
        - Ensure the PIDRotate command works as expected (don't spend too much time debugging; just get it do turn and shoot somewhere else)
        - Ideally we get the shot to face the hangar zone (but for this first comp the hangar zone isn't even in the right spot, doesn't matter)
    */
      new InstantCommand(() -> swerve.zeroHeading()),
      new AutoShoot(shooter, conveyor, hood, 0, 3000).withTimeout(1.5),
      new ParallelCommandGroup(
        new RunTrajectory(swerve, "oneBallGetOut", true),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new RunIntakeCommand(intake, false)
        )
      ).withTimeout(4),
      new ParallelCommandGroup(
        new PIDRotate(swerve, -90).withTimeout(2),
        new SequentialCommandGroup(
          new WaitCommand(.5),
          new AutoShoot(shooter, conveyor, hood, 25000, 5000).withTimeout(3)
        )
      )
    );
  }
}
