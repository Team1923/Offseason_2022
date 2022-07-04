// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.UnitConversion;
import frc.robot.commands.scoring.independent.RunIntakeCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoonBall extends SequentialCommandGroup {
  /** Creates a new DeuxBall. */
  public GoonBall(SwerveSubsystem swerve, ShooterSubsystem shooter, ConveyorSubsystem conveyor, IntakeSubsystem intake, HoodSubsystem hood, LimelightSubsystem limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> swerve.zeroHeading()),
      new ParallelCommandGroup(
        new RunIntakeCommand(intake, false),
        new RunTrajectory(swerve, "twoBallPath", false)
      ).withTimeout(1.5),
      new PIDRotate(swerve, -180),
      new VisionTrack(swerve, () -> fake(), ()-> fake(), limelight).withTimeout(0.5),
      new AutoShoot(shooter, conveyor, hood, UnitConversion.angleToTicks(27.5), 3300).withTimeout(2),
      new PIDRotate(swerve, -90),
      new ParallelCommandGroup(
        new RunIntakeCommand(intake, false),
        new SequentialCommandGroup(
          new RunTrajectory(swerve, "idk", false),
          new PIDRotate(swerve, -180),
          new RunTrajectory(swerve, "idk", false),
          new PIDRotate(swerve, 90)
        )
        
      )

        
    );
  }

  public double fake(){
    return 0;
  }
}
