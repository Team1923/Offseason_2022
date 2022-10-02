// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

  public WPI_TalonFX hoodMotor = new WPI_TalonFX(HoodConstants.hoodMotorID);


  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    
    hoodMotor.configFactoryDefault();

    hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    
    hoodMotor.configSupplyCurrentLimit(HoodConstants.hoodCurrentLimit);
    hoodMotor.configNominalOutputForward(0.0, Constants.timeoutMs);
    hoodMotor.configNominalOutputReverse(0.0, Constants.timeoutMs);
    hoodMotor.configPeakOutputForward(1.0, Constants.timeoutMs);
    hoodMotor.configPeakOutputReverse(-1.0, Constants.timeoutMs);

    hoodMotor.configAllowableClosedloopError(0, 1);

    hoodMotor.configMotionCruiseVelocity(7500);
    hoodMotor.configMotionAcceleration(15000);



    //configure PID for hood
    setShootConstants();

    resetEncoder();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood Encoder Position: ", getPosition());
    //SmartDashboard.putNumber("Hood Supply Current: ", hoodMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Hood Output Current: ", hoodMotor.getStatorCurrent());
  }

  public void setClimbConstants() {
    
    hoodMotor.configMotionCruiseVelocity(13000);
    hoodMotor.configMotionAcceleration(22000);

    hoodMotor.config_kP(0, HoodConstants.hood_climbkP, Constants.timeoutMs);
    hoodMotor.config_kI(0, HoodConstants.hood_climbkI, Constants.timeoutMs);
    hoodMotor.config_kD(0, HoodConstants.hood_climbkD, Constants.timeoutMs);
    hoodMotor.config_kF(0, HoodConstants.hood_climbkFF, Constants.timeoutMs);
  }

  public void setShootConstants() {
    
    hoodMotor.configMotionCruiseVelocity(7500);
    hoodMotor.configMotionAcceleration(15000);

    hoodMotor.config_kP(0, HoodConstants.hood_shootkP, Constants.timeoutMs);
    hoodMotor.config_kI(0, HoodConstants.hood_shootkI, Constants.timeoutMs);
    hoodMotor.config_kD(0, HoodConstants.hood_shootkD, Constants.timeoutMs);
    hoodMotor.config_kF(0, HoodConstants.hood_shootkFF, Constants.timeoutMs);
  }

  public void setHoodPosition(double setpoint) {
    hoodMotor.set(ControlMode.MotionMagic, setpoint);
  }

  public void set(double output) {
    hoodMotor.set(ControlMode.PercentOutput, output);
  }

  public void stop() {
    hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncoder() {
    hoodMotor.setSelectedSensorPosition(HoodConstants.zeroPosition);
  }

  public void hoodAbort(){
    hoodMotor.setSelectedSensorPosition(-50);
  }

  public double getPosition() {
    return hoodMotor.getSelectedSensorPosition();
  }

  public void playMusic() {

  }
  
}
