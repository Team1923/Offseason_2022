// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import frc.robot.Constants;

/** Class for converting native units to more useful things */
public class UnitConversion {

    // // Motor Ticks --> Steering Wheel Turns 
    // public static double ticksToRotations(double ticks) {
    //     return ticks/Constants.ticks_per_spin;
    // }

    // // Steering Wheel Turns --> Motor Ticks
    // public static double rotationsToTicks(double rotations) {
    //     return rotations*Constants.ticks_per_spin;
    // }

    // // Motor Ticks --> Steering Wheel Total Radians
    // public static double ticksToRadians(double ticks) {
    //     return (ticks/Constants.ticks_per_spin)*2*Math.PI;
    // }

    // // Motor Ticks --> Steering Wheel Bounded Radians [0,2pi]
    // public static double ticksToRadianBounded(double ticks) {
    //     return (ticks%(Constants.ticks_per_spin))*2*Math.PI;
    // }

    // // Steering Wheel Radians --> Motor Ticks
    // public static double radiansToTicks(double radians) {
    //     return (radians/(2*Math.PI))*Constants.ticks_per_spin;
    // }

     // velocity, FalconFX.
  public static double nativeUnitstoRPM(double nativeUnits) {
    return (nativeUnits * 600) / Constants.ticksPerRev;
  }

  public static double RPMtoNativeUnits(double RPM) {
    return (RPM / 600) * Constants.ticksPerRev;
  }

  // position, FalconFX.
  public static double positionRotsToNativeUnits(double rots) {
    return rots * Constants.ticksPerRev;
  }

  public static double positionNativeToRots(double nativeUnits) {
    return nativeUnits / Constants.ticksPerRev;
  }

  public static double inchesToMeters(double inches) {
    return inches / 39.37;
  }

  public static double angleToTicks(double angle){
    return angle * (39835.0 / 83.5);
  }

    
}
