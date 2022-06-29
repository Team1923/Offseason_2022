// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateHandler;
import frc.robot.DesiredClimb.Climbs;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  public ClimbSequence(Climbs climbSequence, ClimbSubsystem climb, HoodSubsystem hood, StateHandler stateHandler, MKIPicoColorSensor colorSensor, Joystick operatorJoystick, Joystick driverJoystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    
    addCommands(
        

    );
  }
}
