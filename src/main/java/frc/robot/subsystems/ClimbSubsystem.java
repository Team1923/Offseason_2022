// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utilities.DesiredClimb;
public class ClimbSubsystem extends SubsystemBase {

  private TalonFX leftClimber;
  private TalonFX rightClimber;
  
  public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 60, 60, 12);

  private DesiredClimb desiredClimb;
  private Joystick operatorJoystick;

  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");

  ShuffleboardLayout climbLayout = coachTab.getLayout("Climb", "List Layout").withPosition(1, 2).withSize(1, 1);

  NetworkTableEntry currentState = climbLayout.add("Current Climb", "FULL_TRAVERSAL").getEntry();

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Joystick oJoystick, DesiredClimb d, HoodSubsystem hood, Joystick driver) {
    
    leftClimber = new TalonFX(17);
    rightClimber = new TalonFX(18);

    leftClimber.configFactoryDefault();
    rightClimber.configFactoryDefault();

    rightClimber.follow(leftClimber);

    rightClimber.setInverted(InvertType.OpposeMaster);

    leftClimber.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
    leftClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    leftClimber.configNominalOutputForward(0, Constants.timeoutMs);
    leftClimber.configNominalOutputReverse(0, Constants.timeoutMs);
    leftClimber.configPeakOutputForward(1, Constants.timeoutMs);
    leftClimber.configPeakOutputReverse(-1, Constants.timeoutMs);
    leftClimber.configAllowableClosedloopError(0, 1);
    leftClimber.configMotionCruiseVelocity(30000); //45000
    leftClimber.configMotionAcceleration(30000); // 60000
    leftClimber.config_kP(0, ClimbConstants.arm_kP, Constants.timeoutMs);
    leftClimber.config_kI(0, ClimbConstants.arm_kI, Constants.timeoutMs);
    leftClimber.config_kD(0, ClimbConstants.arm_kD, Constants.timeoutMs);
    leftClimber.config_kF(0, ClimbConstants.arm_kF, Constants.timeoutMs);
    

    leftClimber.configVoltageCompSaturation(12, Constants.timeoutMs);
    rightClimber.configVoltageCompSaturation(12, Constants.timeoutMs);

    leftClimber.enableVoltageCompensation(true);
    rightClimber.enableVoltageCompensation(true);

    rightClimber.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
    rightClimber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    rightClimber.configNominalOutputForward(0.0, Constants.timeoutMs);
    rightClimber.configNominalOutputReverse(0.0, Constants.timeoutMs);
    rightClimber.configPeakOutputForward(1, Constants.timeoutMs);
    rightClimber.configPeakOutputReverse(-1, Constants.timeoutMs);
    rightClimber.configAllowableClosedloopError(0, 1);
    rightClimber.configMotionCruiseVelocity(30000);
    rightClimber.configMotionAcceleration(30000);
    rightClimber.config_kP(0, ClimbConstants.arm_kP, Constants.timeoutMs);
    rightClimber.config_kI(0, ClimbConstants.arm_kI, Constants.timeoutMs);
    rightClimber.config_kD(0, ClimbConstants.arm_kD, Constants.timeoutMs);
    rightClimber.config_kF(0, ClimbConstants.arm_kF, Constants.timeoutMs);

    this.operatorJoystick = oJoystick;
    this.desiredClimb = d;

    resetEncoders();
  }

  @Override
  public void periodic() {
    
    desiredClimb.updateCurrentClimb(operatorJoystick);

    currentState.setString(desiredClimb.getCurrentClimb().toString());
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
    leftClimber.set(ControlMode.MotionMagic, setpoint);
  }
  public void stop() {
    leftClimber.set(ControlMode.PercentOutput, 0);
  }

  public void set(double output) {
    leftClimber.set(ControlMode.PercentOutput, output);
  }

}
