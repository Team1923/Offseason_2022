// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitConversion;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  private WPI_TalonFX leftShooterMotor = new WPI_TalonFX(ShooterConstants.leftShooterMotorID);
  private WPI_TalonFX rightShooterMotor = new WPI_TalonFX(ShooterConstants.rightShooterMotorID);
  private WPI_TalonFX hoodMotor = new WPI_TalonFX(ShooterConstants.hoodMotorID);

  public ShooterSubsystem() {
    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();
    hoodMotor.configFactoryDefault();

    //follow the right motor
    rightShooterMotor.follow(leftShooterMotor);

    //set inverts
    leftShooterMotor.setInverted(InvertType.None);
    rightShooterMotor.setInverted(InvertType.InvertMotorOutput);

    //current limit stuff
    leftShooterMotor.configSupplyCurrentLimit(ShooterConstants.shooterCurrentLimit);
    rightShooterMotor.configSupplyCurrentLimit(ShooterConstants.shooterCurrentLimit);
    hoodMotor.configSupplyCurrentLimit(ShooterConstants.hoodCurrentLimit);

    //nominal output stuff
    leftShooterMotor.configNominalOutputForward(0, 30);
    rightShooterMotor.configNominalOutputForward(0, 30);
    hoodMotor.configNominalOutputForward(0, 30);

    leftShooterMotor.configNominalOutputReverse(0, 30);
    rightShooterMotor.configNominalOutputReverse(0, 30);
    hoodMotor.configNominalOutputReverse(0, 30);

    //peak output stuff
    leftShooterMotor.configPeakOutputForward(1, 30);
    rightShooterMotor.configPeakOutputForward(1, 30);
    hoodMotor.configPeakOutputForward(1, 30);

    leftShooterMotor.configPeakOutputReverse(-1, 30);
    rightShooterMotor.configPeakOutputReverse(-1, 30);
    hoodMotor.configPeakOutputReverse(-1, 30);

    //configure sensors on the falcons
    leftShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    rightShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);

    //config PID for shooter wheel motors
    leftShooterMotor.config_kP(0, ShooterConstants.shooterkP, Constants.timeoutMs);
    leftShooterMotor.config_kI(0, ShooterConstants.shooterkI, Constants.timeoutMs);
    leftShooterMotor.config_kD(0, ShooterConstants.shooterkD, Constants.timeoutMs);
    leftShooterMotor.config_kF(0, ShooterConstants.shooterkFF, Constants.timeoutMs);

    rightShooterMotor.config_kP(0, ShooterConstants.shooterkP, Constants.timeoutMs);
    rightShooterMotor.config_kI(0, ShooterConstants.shooterkI, Constants.timeoutMs);
    rightShooterMotor.config_kD(0, ShooterConstants.shooterkD, Constants.timeoutMs);
    rightShooterMotor.config_kF(0, ShooterConstants.shooterkFF, Constants.timeoutMs);

    //configure PID for hood
    setShootConstants();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Encoder Position: ", getHoodPosition());
    SmartDashboard.putNumber("Hood Supply Current: ", hoodMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Hood Output Current: ", hoodMotor.getStatorCurrent());
  }

  public void setClimbConstants() {
    hoodMotor.config_kP(0, ShooterConstants.hood_climbkP, Constants.timeoutMs);
    hoodMotor.config_kI(0, ShooterConstants.hood_climbkI, Constants.timeoutMs);
    hoodMotor.config_kD(0, ShooterConstants.hood_climbkD, Constants.timeoutMs);
    hoodMotor.config_kF(0, ShooterConstants.hood_climbkFF, Constants.timeoutMs);
  }

  public void setShootConstants() {
    hoodMotor.config_kP(0, ShooterConstants.hood_shootkP, Constants.timeoutMs);
    hoodMotor.config_kI(0, ShooterConstants.hood_shootkI, Constants.timeoutMs);
    hoodMotor.config_kD(0, ShooterConstants.hood_shootkD, Constants.timeoutMs);
    hoodMotor.config_kF(0, ShooterConstants.hood_shootkFF, Constants.timeoutMs);
  }

  public void setShooterWheelsRPM(double vel) {
    leftShooterMotor.set(ControlMode.Velocity, UnitConversion.RPMtoNativeUnits(vel));
  }

  public void stopShooterWheels() {
    leftShooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setHoodPosition(double setpoint) {
    hoodMotor.set(ControlMode.MotionMagic, setpoint);
  }

  public void stopHood() {
    hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncoders() {
    leftShooterMotor.setSelectedSensorPosition(0);
    rightShooterMotor.setSelectedSensorPosition(0);
    hoodMotor.setSelectedSensorPosition(0);
  }

  public double getHoodPosition() {
    return hoodMotor.getSelectedSensorPosition();
  }
}
