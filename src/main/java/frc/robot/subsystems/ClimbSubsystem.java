// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private CANSparkMax leftClimber;
  private CANSparkMax rightClimber;

  private RelativeEncoder leftClimbEncoder;
  private RelativeEncoder rightClimbEncoder;

  private SparkMaxPIDController leftClimbPIDController;
  private SparkMaxPIDController rightClimbPIDController;


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    leftClimber = new CANSparkMax(17, MotorType.kBrushless);
    rightClimber = new CANSparkMax(18, MotorType.kBrushless);

    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimbEncoder = leftClimber.getEncoder();
    rightClimbEncoder = rightClimber.getEncoder();

    leftClimbPIDController = leftClimber.getPIDController();
    rightClimbPIDController = rightClimber.getPIDController();

    

    resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climb Encoder Position: ", getLeftClimbEncoderPosition());
    SmartDashboard.putNumber("Right Climb Encoder Position: ", getRightClimbEncoderPosition());
  }

  public void resetEncoder() {
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);
  }

  public double getLeftClimbEncoderPosition() {
    return leftClimbEncoder.getPosition();
  } 

  public double getRightClimbEncoderPosition() {
    return rightClimbEncoder.getPosition();
  }

}
