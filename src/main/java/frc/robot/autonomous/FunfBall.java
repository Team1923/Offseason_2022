// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateHandler;
import frc.robot.UnitConversion;
import frc.robot.commands.climb.HoodSingleSetpointCommand;
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
public class FunfBall extends SequentialCommandGroup {
  /** Creates a new FunfBall. */
  public FunfBall(SwerveSubsystem swerve, HoodSubsystem hood, ConveyorSubsystem conveyor, IntakeSubsystem intake, ShooterSubsystem shooter, LimelightSubsystem limelight, StateHandler stateHandler) {
    addCommands(
      new ParallelCommandGroup(
        new HoodSingleSetpointCommand(hood, 0),
        new RunIntakeCommand(intake, false),
        new RunTrajectory(swerve, "twoBallPath", false)
      ).withTimeout(2),
      new PIDRotateN(swerve, -180, false).withTimeout(2),
      new VisionTrack(swerve, () -> fake(), ()-> fake(), limelight).withTimeout(0.5),
      new AutoShoot(shooter, conveyor, hood, UnitConversion.angleToTicks(24), 3200).withTimeout(1.5),
      new ParallelCommandGroup(
        new RunIntakeCommand(intake, false),
        new RunTrajectory(swerve, "getThirdBall", false)
      ).withTimeout(2),
      new VisionTrack(swerve, () -> fake(), ()-> fake(), limelight).withTimeout(0.5),
      new AutoShoot(shooter, conveyor, hood, UnitConversion.angleToTicks(24), 3200).withTimeout(1),

      new InstantCommand(() -> stateHandler.resetState())

    );
  }

  public double fake(){
    return 0;
  }
}
