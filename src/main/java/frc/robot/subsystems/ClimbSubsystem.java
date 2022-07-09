// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DesiredClimb;
import frc.robot.Constants.ClimbConstants;
public class ClimbSubsystem extends SubsystemBase {

  private TalonFX leftClimber;
  private TalonFX rightClimber;

  private DesiredClimb desiredClimb;
  private Joystick operatorJoystick;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Joystick oJoystick, DesiredClimb d, HoodSubsystem hood, Joystick driver) {
    
    leftClimber = new TalonFX(17);
    rightClimber = new TalonFX(18);

    leftClimber.configFactoryDefault();
    rightClimber.configFactoryDefault();

    leftClimber.follow(rightClimber);

    rightClimber.setInverted(InvertType.OpposeMaster);

    leftClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    
    leftClimber.configNominalOutputForward(0.0, Constants.timeoutMs);
    leftClimber.configNominalOutputReverse(0.0, Constants.timeoutMs);
    leftClimber.configPeakOutputForward(1.0, Constants.timeoutMs);
    leftClimber.configPeakOutputReverse(-1.0, Constants.timeoutMs);

    leftClimber.configAllowableClosedloopError(0, 1);

    leftClimber.config_kP(0, ClimbConstants.arm_kP, Constants.timeoutMs);
    leftClimber.config_kI(0, ClimbConstants.arm_kI, Constants.timeoutMs);
    leftClimber.config_kD(0, ClimbConstants.arm_kD, Constants.timeoutMs);
    leftClimber.config_kF(0, ClimbConstants.arm_kF, Constants.timeoutMs);

    leftClimber.configMotionCruiseVelocity(7500);
    leftClimber.configMotionAcceleration(15000);


    rightClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    
    rightClimber.configNominalOutputForward(0.0, Constants.timeoutMs);
    rightClimber.configNominalOutputReverse(0.0, Constants.timeoutMs);
    rightClimber.configPeakOutputForward(1.0, Constants.timeoutMs);
    rightClimber.configPeakOutputReverse(-1.0, Constants.timeoutMs);

    rightClimber.configAllowableClosedloopError(0, 1);
    
    leftClimber.config_kP(0, ClimbConstants.arm_kP, Constants.timeoutMs);
    leftClimber.config_kI(0, ClimbConstants.arm_kI, Constants.timeoutMs);
    leftClimber.config_kD(0, ClimbConstants.arm_kD, Constants.timeoutMs);
    leftClimber.config_kF(0, ClimbConstants.arm_kF, Constants.timeoutMs);

    rightClimber.configMotionCruiseVelocity(7500);
    rightClimber.configMotionAcceleration(15000);

    this.operatorJoystick = oJoystick;
    this.desiredClimb = d;

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climb Encoder Position: ", getLeftClimbEncoderPosition());
    SmartDashboard.putNumber("Right Climb Encoder Position: ", getRightClimbEncoderPosition());
    
    desiredClimb.updateCurrentClimb(operatorJoystick);
    SmartDashboard.putString("CURRENT CLIMB SEQUENCE", desiredClimb.getCurrentClimb().toString());
    
  }

  public void resetEncoders() {
    leftClimber.setSelectedSensorPosition(0);
    rightClimber.setSelectedSensorPosition(0);
  }

  public double getLeftClimbEncoderPosition() {
    return leftClimber.getSelectedSensorPosition();
  } 

  public double getRightClimbEncoderPosition() {
    return rightClimber.getSelectedSensorPosition();
  }

  public void setPID(double setpoint) {
    rightClimber.set(ControlMode.MotionMagic, setpoint);
  }
  public void stop() {
    leftClimber.set(ControlMode.PercentOutput, 0);
  }

  public void set(double output) {
    rightClimber.set(ControlMode.PercentOutput, output);
  }

}
