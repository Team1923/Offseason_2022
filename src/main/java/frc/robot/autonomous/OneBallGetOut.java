// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      new InstantCommand(() -> swerve.zeroHeading()),
      new AutoShoot(shooter, conveyor, hood, 0, 3000).withTimeout(1.5),
      new ParallelCommandGroup(
        new RunTrajectory(swerve, "oneBallGetOut", true),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new RunIntakeCommand(intake, false)
        )
      ).withTimeout(4),
      new PIDRotate(swerve, -90).withTimeout(2),
      new AutoShoot(shooter, conveyor, hood, 25000, 5000)
    );
  }
}
