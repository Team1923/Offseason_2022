// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DesiredClimb;
import frc.robot.StateHandler;
import frc.robot.Constants.ClimbConstants;
import frc.robot.MKILib.MKIPicoColorSensor;
public class ClimbSubsystem extends SubsystemBase {

  private CANSparkMax leftClimber;
  private CANSparkMax rightClimber;

  private RelativeEncoder leftClimbEncoder;
  private RelativeEncoder rightClimbEncoder;

  private SparkMaxPIDController leftClimbPIDController;
  private SparkMaxPIDController rightClimbPIDController;

  private DesiredClimb desiredClimb;
  private Joystick operatorJoystick;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(Joystick oJoystick, DesiredClimb d, HoodSubsystem hood, Joystick driver, StateHandler state, MKIPicoColorSensor color) {
    
    leftClimber = new CANSparkMax(17, MotorType.kBrushless);
    rightClimber = new CANSparkMax(18, MotorType.kBrushless);

    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();
    
    rightClimber.follow(leftClimber, true);

    leftClimbEncoder = leftClimber.getEncoder();
    rightClimbEncoder = rightClimber.getEncoder();

    leftClimbPIDController = leftClimber.getPIDController();
    rightClimbPIDController = rightClimber.getPIDController();

    leftClimbPIDController.setP(ClimbConstants.arm_kP);
    leftClimbPIDController.setI(ClimbConstants.arm_kI);
    leftClimbPIDController.setD(ClimbConstants.arm_kD);
    leftClimbPIDController.setIZone(ClimbConstants.arm_kIz);
    leftClimbPIDController.setFF(ClimbConstants.arm_kFF);
    leftClimbPIDController.setOutputRange(ClimbConstants.arm_minOutput, ClimbConstants.arm_maxOutput);

    rightClimbPIDController.setP(ClimbConstants.arm_kP);
    rightClimbPIDController.setI(ClimbConstants.arm_kI);
    rightClimbPIDController.setD(ClimbConstants.arm_kD);
    rightClimbPIDController.setIZone(ClimbConstants.arm_kIz);
    rightClimbPIDController.setFF(ClimbConstants.arm_kFF);
    rightClimbPIDController.setOutputRange(ClimbConstants.arm_minOutput, ClimbConstants.arm_maxOutput);

    leftClimbPIDController.setSmartMotionMaxVelocity(ClimbConstants.arm_maxVel, 0);
    leftClimbPIDController.setSmartMotionMinOutputVelocity(ClimbConstants.arm_minVel, 0);
    leftClimbPIDController.setSmartMotionMaxAccel(ClimbConstants.arm_maxAcc, 0);
    leftClimbPIDController.setSmartMotionAllowedClosedLoopError(ClimbConstants.arm_allowedErr, 0);

    rightClimbPIDController.setSmartMotionMaxVelocity(ClimbConstants.arm_maxVel, 0);
    rightClimbPIDController.setSmartMotionMinOutputVelocity(ClimbConstants.arm_minVel, 0);
    rightClimbPIDController.setSmartMotionMaxAccel(ClimbConstants.arm_maxAcc, 0);
    rightClimbPIDController.setSmartMotionAllowedClosedLoopError(ClimbConstants.arm_allowedErr, 0);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    this.operatorJoystick = oJoystick;
    this.desiredClimb = d;

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climb Encoder Position: ", getLeftClimbEncoderPosition());
    SmartDashboard.putNumber("Right Climb Encoder Position: ", getRightClimbEncoderPosition());

    //SmartDashboard.putNumber("Left Climber Current Draw: ", getLeftClimbCurrentDraw());
    //SmartDashboard.putNumber("Right Climber Current Draw: ", getRightClimbCurrentDraw());

    desiredClimb.updateCurrentClimb(operatorJoystick);
    SmartDashboard.putString("CURRENT CLIMB SEQUENCE", desiredClimb.getCurrentClimb().toString());

    // switch(desiredClimb.getCurrentClimb()){
    //   case FULL_TRAVERSAL:
    //     new JoystickButton(operatorJoystick, OIConstants.kOperatorLeftBumper).toggleWhenPressed(new FullTraversalClimbSequence(HOOD_SUBSYSTEM, this, () -> driverJoystick.getRawButton(5), stateHandler, colorSensor));
    //     break;
    //   case TRAVERSAL_ARMS_EXTENDED:
    //     new JoystickButton(operatorJoystick, OIConstants.kOperatorLeftBumper).toggleWhenPressed(new TraversalArmsExtended(HOOD_SUBSYSTEM, this, () -> driverJoystick.getRawButton(5), stateHandler, colorSensor));
    //     break;
    //   case LEVEL_THREE:
    //     new JoystickButton(operatorJoystick, OIConstants.kOperatorLeftBumper).toggleWhenPressed(new LevelThreeClimb(HOOD_SUBSYSTEM, this, () -> driverJoystick.getRawButton(5), stateHandler, colorSensor));
    //     break;
    //   case LEVEL_TWO:
    //     new JoystickButton(operatorJoystick, OIConstants.kOperatorLeftBumper).toggleWhenPressed(new LevelTwoClimb(HOOD_SUBSYSTEM, this, () -> driverJoystick.getRawButton(5), stateHandler, colorSensor));
    //     break;
    //   default:
    //     break;
    // }
    
  }

  public void resetEncoders() {
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);
  }

  public double getLeftClimbEncoderPosition() {
    return leftClimbEncoder.getPosition();
  } 

  public double getRightClimbEncoderPosition() {
    return rightClimbEncoder.getPosition();
  }

  public void setPID(double setpoint) {
    leftClimbPIDController.setReference(setpoint, ControlType.kSmartMotion);
  }

  public double getLeftClimbCurrentDraw() {
    return leftClimber.getOutputCurrent();
  }

  public double getRightClimbCurrentDraw() {
    return rightClimber.getOutputCurrent();
  }

  public void stop() {
    rightClimber.set(0);
    leftClimber.set(0);
  }

  public void set(double output) {
    leftClimber.set(output);
  }

}
