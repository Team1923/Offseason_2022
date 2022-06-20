// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.interfaces.LimelightInterface;

@SuppressWarnings("unused")
public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private LimelightInterface Limelight;
  

  // height in inches of camera from ground
  private double limelight_height = 20;
  // height in inches of center of target from ground
  private double target_height = 100;
  // limelight mounting angle above positive x axis in degrees
  private double limelight_mount_angle = 30;

  public LimelightSubsystem(LimelightInterface limelight) {    
    Limelight = limelight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("x angle: ", getX());
    SmartDashboard.putNumber("y angle:", getY());
    SmartDashboard.putNumber("area: ", getArea());

  }

  public double getX() {
    return Limelight.getHorizontalOffset();
  }

  public double getY() {
    return Limelight.getVerticalOffset();
  }

  public double getArea() {
    return Limelight.getArea();
  }

  

  public double map(double a1, double a2, double b1, double b2, double input) {
    return b1 + ((input-a1)*(b2-b1)/(a2-a1));
  }

  public double angle() {
    return limelight_mount_angle - getY();
  }

}
