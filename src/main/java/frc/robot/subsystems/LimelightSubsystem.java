// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.LimelightInterface;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private LimelightInterface Limelight;
  public boolean isGoalCentric;

  public LimelightSubsystem(LimelightInterface limelight) {    
    Limelight = limelight;
    isGoalCentric = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("x angle: ", getX());
    //SmartDashboard.putNumber("y angle:", getY());
    //SmartDashboard.putNumber("area: ", getArea());

    SmartDashboard.putNumber("Distance", getDistance());

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

  public double getDistance(){
    return Limelight.distanceToTarget();
  }

  public boolean hasTarget(){
    return Limelight.validTargets();
  }

  public void setIsGoalCentric(boolean bool){
    isGoalCentric = bool;
  }

  public boolean getIsGoalCentric(){
    return isGoalCentric;
  }

}
