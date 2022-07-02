// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConveyorConstants;
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
      new AutoShoot(shooter, conveyor, hood, 5000, 3200).withTimeout(1.5),
      new RunTrajectory(swerve, "oneBallGetOut")
    );
  }
}
