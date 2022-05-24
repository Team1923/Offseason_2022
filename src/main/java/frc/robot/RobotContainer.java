// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystem Instances
  private final SwerveSubsystem SWERVE_SUBSYSTEM = new SwerveSubsystem();

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
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); // By default, since "!" is included, field oriented is default

  }

  // Define what buttons will do
  private void configureButtonBindings() {
    // Creates an "instant command" that will execute the line of code past the "() ->"
    new JoystickButton(driverJoystick, 2).whenPressed(() -> SWERVE_SUBSYSTEM.zeroHeading());
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup();
  }
}
