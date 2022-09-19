// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitConversion;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  public WPI_TalonFX leftShooterMotor = new WPI_TalonFX(ShooterConstants.leftShooterMotorID);
  public WPI_TalonFX rightShooterMotor = new WPI_TalonFX(ShooterConstants.rightShooterMotorID);

  private boolean acceptableRPM;
  private boolean shooterStalling;

  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");

  ShuffleboardLayout shooterLayout = coachTab.getLayout("Shooter", "List Layout").withPosition(3, 0).withSize(1, 2);

  private NetworkTableEntry isWithinRange = shooterLayout.add("RPM GOAL:", false)
  .withSize(1, 1)
  .withPosition(0, 4)
  .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
  .getEntry();

  private NetworkTableEntry currentShooterRPM = shooterLayout.add("SHOOTER RPM", 0).getEntry();

  public ShooterSubsystem() {
    
    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();

    //follow the right motor
    rightShooterMotor.follow(leftShooterMotor);

    //set inverts
    leftShooterMotor.setInverted(InvertType.None);
    rightShooterMotor.setInverted(InvertType.None);

    //current limit stuff
    leftShooterMotor.configSupplyCurrentLimit(ShooterConstants.shooterCurrentLimit);
    rightShooterMotor.configSupplyCurrentLimit(ShooterConstants.shooterCurrentLimit);

    //nominal output stuff
    leftShooterMotor.configNominalOutputForward(0, Constants.timeoutMs);
    rightShooterMotor.configNominalOutputForward(0, Constants.timeoutMs);

    leftShooterMotor.configNominalOutputReverse(0, Constants.timeoutMs);
    rightShooterMotor.configNominalOutputReverse(0, Constants.timeoutMs);

    //peak output stuff
    leftShooterMotor.configPeakOutputForward(1, Constants.timeoutMs);
    rightShooterMotor.configPeakOutputForward(1, Constants.timeoutMs);

    leftShooterMotor.configPeakOutputReverse(-1, Constants.timeoutMs);
    rightShooterMotor.configPeakOutputReverse(-1, Constants.timeoutMs);

    //configure sensors on the falcons
    leftShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    rightShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);

    //config PID for shooter wheel motors
    leftShooterMotor.config_kP(0, ShooterConstants.shooterkP, Constants.timeoutMs);
    leftShooterMotor.config_kI(0, ShooterConstants.shooterkI, Constants.timeoutMs);
    leftShooterMotor.config_kD(0, ShooterConstants.shooterkD, Constants.timeoutMs);
    leftShooterMotor.config_kF(0, ShooterConstants.shooterkFF, Constants.timeoutMs);

    rightShooterMotor.config_kP(0, ShooterConstants.shooterkP, Constants.timeoutMs);
    rightShooterMotor.config_kI(0, ShooterConstants.shooterkI, Constants.timeoutMs);
    rightShooterMotor.config_kD(0, ShooterConstants.shooterkD, Constants.timeoutMs);
    rightShooterMotor.config_kF(0, ShooterConstants.shooterkFF, Constants.timeoutMs);

    acceptableRPM = false;

    resetEncoders();

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM: ", getShooterRPM());
    SmartDashboard.putBoolean("boolean acceptable thing", acceptableRPM);

    isWithinRange.setBoolean(acceptableRPM);
    currentShooterRPM.setDouble(getShooterRPM());
  }

  public double getShooterRPM() {
    return UnitConversion.nativeUnitstoRPM(leftShooterMotor.getSelectedSensorVelocity());
  }

  public void setShooterWheelsRPM(double vel) {
    leftShooterMotor.set(ControlMode.Velocity, UnitConversion.RPMtoNativeUnits(vel));
  }

  public void set(double speed) {
    leftShooterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    leftShooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncoders() {
    leftShooterMotor.setSelectedSensorPosition(0);
    rightShooterMotor.setSelectedSensorPosition(0);
  }

  public void setAcceptableRPMState(boolean acceptable) {
    this.acceptableRPM = acceptable;
  }

  public boolean getAcceptableRPMState() {
    return acceptableRPM;
  }

  public boolean getStalling() {
    return shooterStalling;
  }

  public void setStalling(boolean set) {
    this.shooterStalling = set;
  }

}
