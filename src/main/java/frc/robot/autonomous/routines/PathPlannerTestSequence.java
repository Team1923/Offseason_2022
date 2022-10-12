// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.autonomous.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.commands.scoring.independent.RunIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerTestSequence extends SequentialCommandGroup {
  /** Creates a new PathPlannerTestSequence. */


  public PathPlannerTestSequence(SwerveSubsystem swerve, IntakeSubsystem intake) { 
    
    final AutoFromPathPlanner testAuto = new AutoFromPathPlanner(swerve, "testPath", 3.1, true);

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(testAuto.getInitialPose())),
      new ParallelCommandGroup(
        testAuto,
        new RunIntakeCommand(intake, false)
      )
    );
  }
}
