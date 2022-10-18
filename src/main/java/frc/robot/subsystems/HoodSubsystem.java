// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {


  public WPI_TalonFX hoodMotor = new WPI_TalonFX(HoodConstants.hoodMotorID);

  // ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");

  // ShuffleboardLayout hoodLayout = coachTab.getLayout("HOOD", "List Layout").withPosition(0, 8).withSize(1, 1);

  // private boolean isHoodDown;

  // private NetworkTableEntry hoodDown = hoodLayout.add("Hood Abort?", false)
  // .withSize(1, 1)
  // .withPosition(0, 8)
  // .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
  // .getEntry();

  


  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {

    // isHoodDown = false;

    
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

    // hoodDown.setBoolean(false);



    //configure PID for hood
    setShootConstants();

    resetEncoder();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboar.putNumber("Hood Encoder Position: ", getPosition());
    //SmartDashboard.putNumber("Hood Supply Current: ", hoodMotor.getSupplyCurrent());
   // SmartDashboar.putNumber("Hood Output Current: ", hoodMotor.getStatorCurrent());

  //  hoodDown.setBoolean(isHoodDown);
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
    // isHoodDown = true;
    hoodMotor.setSelectedSensorPosition(-500);
  }

  public double getPosition() {
    return hoodMotor.getSelectedSensorPosition();
  }

  public void playMusic() {

  }
  
}
