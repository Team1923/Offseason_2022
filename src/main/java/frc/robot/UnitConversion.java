// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class UnitConversion {

    public static double ticksToRotations(double ticks) {
        return ticks/Constants.ticks_per_rotation;
    }

    public static double rotationsToTicks(double rotations) {
        return rotations*Constants.ticks_per_rotation;
    }

    public static double ticksToRadians(double ticks) {
        return (ticks%(Constants.ticks_per_spin))*2*Math.PI;
    }

    
}
