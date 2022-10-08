// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.MKILib.MKISpeaker;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.PIDRotateN;
import frc.robot.autonomous.RunTrajectory;
import frc.robot.commands.scoring.ShootCommandGroup;
import frc.robot.commands.climb.ResetArms;
import frc.robot.commands.climb.ScheduleClimb;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.commands.scoring.DefaultHoodCommand;
import frc.robot.commands.scoring.HoodResetHold;
import frc.robot.commands.scoring.RunShooterCommand;
import frc.robot.commands.scoring.ShooterAvoidStallCommand;
import frc.robot.commands.scoring.StateManagedConveyorCommand;
import frc.robot.commands.scoring.StateManagedIntakeCommand;
import frc.robot.interfaces.LimelightInterface;
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
public class RobotContainer {

  private final LimelightInterface Limelight = new LimelightInterface();
  private DesiredClimb desiredClimb = new DesiredClimb();

  ShooterData shooterData = new ShooterData();

  // Joystick Instances
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);
 
  // Subsystem Instances
  private final SwerveSubsystem SWERVE_SUBSYSTEM = new SwerveSubsystem(Limelight);
  private final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem(Limelight);
  private final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();

  private final ConveyorSubsystem CONVEYOR_SUBSYSTEM = new ConveyorSubsystem();
  private final HoodSubsystem HOOD_SUBSYSTEM = new HoodSubsystem();
  private final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  
  private final MKISpeaker speaker = new MKISpeaker();

  // Singleton State Handler
  StateHandler stateHandler = new StateHandler(SHOOTER_SUBSYSTEM, INTAKE_SUBSYSTEM, CONVEYOR_SUBSYSTEM, speaker);  
  DesiredClimb climb = new DesiredClimb();

  private final ClimbSubsystem CLIMB_SUBSYSTEM = new ClimbSubsystem(operatorJoystick, climb, HOOD_SUBSYSTEM, driverJoystick);

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
    SHOOTER_SUBSYSTEM.setDefaultCommand(new ShooterAvoidStallCommand(SHOOTER_SUBSYSTEM, ShooterConstants.avoidStallSpeed));
    CONVEYOR_SUBSYSTEM.setDefaultCommand(new StateManagedConveyorCommand(
        CONVEYOR_SUBSYSTEM, 
        SHOOTER_SUBSYSTEM, 
        stateHandler, 
        () -> SHOOTER_SUBSYSTEM.leftShooterMotor.getStatorCurrent() > 100));

    HOOD_SUBSYSTEM.setDefaultCommand(new DefaultHoodCommand(HOOD_SUBSYSTEM));    

    
  }

  // Define what buttons will do
  private void configureButtonBindings() {

    //new JoystickButton(operatorJoystick, OIConstants.kOperatorLeftBumper).whenPressed(new ScheduleClimb(desiredClimb, HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, () -> driverJoystick.getRawButton(5), CONVEYOR_SUBSYSTEM, INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM, SWERVE_SUBSYSTEM));
    
    new JoystickButton(operatorJoystick, OIConstants.kOperatorRightBumper).whileHeld(new ResetArms(CLIMB_SUBSYSTEM));
 
    // Run intake command
    new JoystickButton(operatorJoystick, OIConstants.kOperatorXButton).whileHeld(new StateManagedIntakeCommand(INTAKE_SUBSYSTEM, stateHandler, CONVEYOR_SUBSYSTEM, false));
    new JoystickButton(operatorJoystick, OIConstants.kOperatorSquareButton).whileHeld(new StateManagedIntakeCommand(INTAKE_SUBSYSTEM, stateHandler, CONVEYOR_SUBSYSTEM, true));
    
    // Creates an "instant command" that will execute the line of code past the "() ->", zeros the heading
    new JoystickButton(driverJoystick, OIConstants.kDriverBButton).whenPressed(() -> SWERVE_SUBSYSTEM.zeroHeading());

    //reset heading button
    new JoystickButton(operatorJoystick, OIConstants.kOperatorCircleButton).whenPressed(() -> stateHandler.resetState());

    new JoystickButton(operatorJoystick, OIConstants.kOperatorTriangleButton).whileHeld(new RunShooterCommand(SHOOTER_SUBSYSTEM, CONVEYOR_SUBSYSTEM, shooterData, LIMELIGHT_SUBSYSTEM));

    // Hub-centric driving command

    new JoystickButton(driverJoystick, OIConstants.kDriverXButton).whenPressed(() -> HOOD_SUBSYSTEM.hoodAbort());

    // CONFIRM AXIS 3 IS THE CORRECT RIGHT TRIGGER FOR OPERATOR
    //new Trigger(() -> operatorJoystick.getRawAxis(3) > .2).whenActive(new HoodResetHold(HOOD_SUBSYSTEM), false);

    new Trigger(() -> driverJoystick.getRawAxis(3) > .1).whileActiveContinuous(new ShootCommandGroup(
        SWERVE_SUBSYSTEM, 
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis), 
        LIMELIGHT_SUBSYSTEM, HOOD_SUBSYSTEM, shooterData));

    new JoystickButton(operatorJoystick, OIConstants.kOperatorLeftBumper).whenPressed(new ScheduleClimb(desiredClimb, HOOD_SUBSYSTEM, CLIMB_SUBSYSTEM, () -> driverJoystick.getRawButton(5), CONVEYOR_SUBSYSTEM, INTAKE_SUBSYSTEM, SHOOTER_SUBSYSTEM, SWERVE_SUBSYSTEM));
    
    new JoystickButton(operatorJoystick, OIConstants.kOperatorRightBumper).whileHeld(new ResetArms(CLIMB_SUBSYSTEM));


    //new JoystickButton(operatorJoystick, OIConstants.kOperatorLeftBumper).whileHeld(new ClimbApplyVoltage(CLIMB_SUBSYSTEM, -.25));
    //new JoystickButton(operatorJoystick, OIConstants.kOperatorCircleButton).whenPressed(() -> CLIMB_SUBSYSTEM.resetEncoders());
  }

  // public Command getAutonomousCommand() {
  //   return new UnoBall(SWERVE_SUBSYSTEM, SHOOTER_SUBSYSTEM, CONVEYOR_SUBSYSTEM, INTAKE_SUBSYSTEM, HOOD_SUBSYSTEM);
    
  // }

  public Command initializeAuto(AutoChooser selector) {
    return selector.startMode(SWERVE_SUBSYSTEM, SHOOTER_SUBSYSTEM, CONVEYOR_SUBSYSTEM, INTAKE_SUBSYSTEM, HOOD_SUBSYSTEM, LIMELIGHT_SUBSYSTEM, stateHandler);
  } 

  public void playSound() {
    speaker.playSound(1);
  }
}
