// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private Solenoid solenoid3 = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  private Solenoid solenoid4 = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

  public WPI_TalonFX leftIntakeMotor = new WPI_TalonFX(IntakeConstants.leftIntakeMotorID);
  public WPI_TalonFX rightIntakeMotor = new WPI_TalonFX(IntakeConstants.rightIntakemotorID);
  
  public boolean intake_reverse;

  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");

  ShuffleboardLayout intakeLayout = coachTab.getLayout("Intake", "List Layout").withPosition(0, 0).withSize(1, 1);

  private NetworkTableEntry intakeDown = intakeLayout.add("Intake Down", false)
  .withSize(1, 1)
  .withPosition(0, 4)
  .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
  .getEntry();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    
    setLow();

    //set factory defaults for all motors
    leftIntakeMotor.configFactoryDefault();
    rightIntakeMotor.configFactoryDefault();

    //right motor is the master motor
    rightIntakeMotor.follow(leftIntakeMotor);

    //set left motor inverted, no invert for the right motor
    leftIntakeMotor.setInverted(InvertType.InvertMotorOutput);
    rightIntakeMotor.setInverted(InvertType.None);

    rightIntakeMotor.setNeutralMode(NeutralMode.Brake);

    //supply current limit
    leftIntakeMotor.configSupplyCurrentLimit(IntakeConstants.intakeCurrentLimit);
    rightIntakeMotor.configSupplyCurrentLimit(IntakeConstants.intakeCurrentLimit);


    //configure nominal output
    leftIntakeMotor.configNominalOutputForward(0.0, Constants.timeoutMs); //needs a percentOut and a timer. Just going by what's suggested. 
    rightIntakeMotor.configNominalOutputForward(0.0, Constants.timeoutMs); // "   "
    leftIntakeMotor.configNominalOutputReverse(0.0, Constants.timeoutMs); // "    "
    rightIntakeMotor.configNominalOutputReverse(0.0, Constants.timeoutMs); // "   "

    //configure peak output(forward: 1, reverse, -1)
    leftIntakeMotor.configPeakOutputForward(1, Constants.timeoutMs);
    rightIntakeMotor.configPeakOutputForward(1, Constants.timeoutMs);
    leftIntakeMotor.configPeakOutputReverse(-1, Constants.timeoutMs);
    leftIntakeMotor.configPeakOutputReverse(-1, Constants.timeoutMs);

    intake_reverse = false;

  }

  @Override
  public void periodic() {
   // SmartDashboar.putBoolean("SOLENOID 1", solenoid1.get());
   // SmartDashboar.putBoolean("SOLENOID 2", solenoid2.get());
   // SmartDashboar.putBoolean("SOLENOID 3", solenoid3.get());
   // SmartDashboar.putBoolean("SOLENOID 4", solenoid4.get());
    
  }


  public void setHigh() {
    solenoid1.set(false);
    solenoid2.set(true);
    solenoid3.set(true);
    solenoid4.set(false);
    intakeDown.setBoolean(true);
  }

  public void setLow() {
    solenoid1.set(true); 
    solenoid2.set(false);  
    solenoid3.set(false);  
    solenoid4.set(true); 
    intakeDown.setBoolean(false); 
  }

  public void setIntake(double percentOut) {
    leftIntakeMotor.set(ControlMode.PercentOutput, -percentOut);
  }

  public void stop() {
    leftIntakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getIntakeCommandedDirection() {
    return intake_reverse;
  }
  
}
