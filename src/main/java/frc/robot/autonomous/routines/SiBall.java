// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.autocommands.AutoShoot;
import frc.robot.autonomous.autocommands.PIDRotateN;
import frc.robot.autonomous.autocommands.RunTrajectory;
import frc.robot.autonomous.autocommands.VisionTrack;
import frc.robot.commands.scoring.independent.RunIntakeCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.UnitConversion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SiBall extends SequentialCommandGroup {
  /** Creates a new DeuxBall. */
  public SiBall(SwerveSubsystem swerve, ShooterSubsystem shooter, ConveyorSubsystem conveyor, IntakeSubsystem intake, HoodSubsystem hood, LimelightSubsystem limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(() -> SmartDashboard.putBoolean("AUTO DONE", false)),
        new RunIntakeCommand(intake, false),
        new RunTrajectory(swerve, "getTwoFor4Ball", false)
      ).withTimeout(2),
      new PIDRotateN(swerve, -177, false).withTimeout(1.5),
      new VisionTrack(swerve, () -> fake(), ()-> fake(), limelight).withTimeout(0.5),
      new AutoShoot(shooter, conveyor, hood, UnitConversion.angleToTicks(24), 3100).withTimeout(1.5),
      new ParallelRaceGroup(
        new RunIntakeCommand(intake, false),
        new RunTrajectory(swerve, "getTwoMore", true)
      ),
      new RunIntakeCommand(intake, false).withTimeout(1.5),
      new RunTrajectory(swerve, "getBack", true),
      new VisionTrack(swerve, () -> fake(), ()-> fake(), limelight).withTimeout(0.5),
      new AutoShoot(shooter, conveyor, hood, UnitConversion.angleToTicks(24), 3100).withTimeout(1.5),
      new InstantCommand(() -> SmartDashboard.putBoolean("AUTO DONE", true))  
    );
  }

  public double fake(){
    return 0;
  }
}
