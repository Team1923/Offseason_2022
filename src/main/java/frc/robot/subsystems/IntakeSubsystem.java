// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private Solenoid solenoid3 = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  private Solenoid solenoid4 = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

  private WPI_TalonFX leftIntakeMotor = new WPI_TalonFX(IntakeConstants.leftIntakeMotorID);
  private WPI_TalonFX rightIntakeMotor = new WPI_TalonFX(IntakeConstants.rightIntakemotorID);
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    setLow();

    //set factory defaults for all motors
    leftIntakeMotor.configFactoryDefault();
    rightIntakeMotor.configFactoryDefault();

    //right motor is the master motor
    leftIntakeMotor.follow(rightIntakeMotor);

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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setHigh() {
    solenoid1.set(true);
    solenoid2.set(false);
    solenoid3.set(false);
    solenoid4.set(true);
  }

  public void setLow() {
    solenoid1.set(false);
    solenoid2.set(true);
    solenoid3.set(true);
    solenoid4.set(false);
  }

  public void setIntake(double percentOut) {
    rightIntakeMotor.set(ControlMode.PercentOutput, percentOut);
  }

  public void stop() {
    rightIntakeMotor.set(ControlMode.PercentOutput, 0);
  }

  
}
