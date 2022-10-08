// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.GoalCentricCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.ShooterData;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommandGroup extends ParallelCommandGroup {
  /** Creates a new ShootCommandGroup. */
  public ShootCommandGroup(SwerveSubsystem swerve, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> ts,
  LimelightSubsystem limelight, HoodSubsystem hood, ShooterData shooterData) {
    
    addCommands(
        new GoalCentricCommand(swerve, xSpdFunction, ySpdFunction, ts, limelight),
        new HoodChangingSetpointCommand(hood, limelight, shooterData)
    );
  }
}
