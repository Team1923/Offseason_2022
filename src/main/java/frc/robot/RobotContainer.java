// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.FollowTrajectory;
import frc.robot.commands.drive.GoalCentricCommand;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.scoring.ShooterAvoidStallCommand;
import frc.robot.commands.scoring.independent.RunIntakeCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// Added in the line below because subsystems not being accessed in this class
// does not mean that they are actually not being used. When defined, the
// command scheduler will automatically schedule the periodic loop defined in them.
@SuppressWarnings("unused")
public class RobotContainer {

 
  // Subsystem Instances
  private final SwerveSubsystem SWERVE_SUBSYSTEM = new SwerveSubsystem();
  private final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  private final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  private final ClimbSubsystem CLIMB_SUBSYSTEM = new ClimbSubsystem();
  private final ConveyorSubsystem CONVEYOR_SUBSYSTEM = new ConveyorSubsystem();
  private final HoodSubsystem HOOD_SUBSYSTEM = new HoodSubsystem();
  private final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();

  // Singleton State Handler
  StateHandler stateHandler = new StateHandler(SHOOTER_SUBSYSTEM, INTAKE_SUBSYSTEM, CONVEYOR_SUBSYSTEM);  


  // Joystick Instances
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set the default commands
    setDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

  }

  // Set the default commands for all subsystems
  private void setDefaultCommands() {
    
    // Sets the default command for the swerve subsystem. Uses some fancy passthrough logic that I don't 
    // understand because I am not fluent in Java, it was given to me by 0-2-Autonomous
    SWERVE_SUBSYSTEM.setDefaultCommand(new SwerveDriveCommand(
        SWERVE_SUBSYSTEM,
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), // Forward and Back
        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), // Side to Side
        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis), // Rotation
        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); // Field oriented control vs robot oriented. By default, since "!" is included, field oriented is default

    // Sets the default command of the shooter to the mode where it trys to spin at a low speed to avoid a stall.
    SHOOTER_SUBSYSTEM.setDefaultCommand(new ShooterAvoidStallCommand(SHOOTER_SUBSYSTEM));    
  }

  // Define what buttons will do
  private void configureButtonBindings() {

    // Run intake command
    new JoystickButton(driverJoystick, OIConstants.kOperatorXButton).whileHeld(new RunIntakeCommand(INTAKE_SUBSYSTEM, false));
    
    // Creates an "instant command" that will execute the line of code past the "() ->", zeros the heading
    new JoystickButton(driverJoystick, OIConstants.kOperatorBButton).whenPressed(() -> SWERVE_SUBSYSTEM.zeroHeading());

    // Hub-centric driving command
    new JoystickButton(driverJoystick, OIConstants.kOperatorAButton).whileHeld(new GoalCentricCommand(
      SWERVE_SUBSYSTEM, 
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), 
      LIMELIGHT_SUBSYSTEM));

  }

  public Command getAutonomousCommand() {
    return new FollowTrajectory(SWERVE_SUBSYSTEM, "2m fwd");
  }

  public void updateBooleans() {

  }
}
