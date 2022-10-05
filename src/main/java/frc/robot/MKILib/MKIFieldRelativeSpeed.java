// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MKILib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Credit to 1706 for the base code for this class. We modified it to fit our needs but the idea and basic functionality came from them.
public class MKIFieldRelativeSpeed {
    public double vx;
    public double vy;
    public double omega;

    public MKIFieldRelativeSpeed(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public MKIFieldRelativeSpeed(ChassisSpeeds chassisSpeed, Rotation2d gyro) {
        this(chassisSpeed.vxMetersPerSecond * gyro.getCos() - chassisSpeed.vyMetersPerSecond * gyro.getSin(),
                chassisSpeed.vyMetersPerSecond * gyro.getCos() + chassisSpeed.vxMetersPerSecond * gyro.getSin(),
                chassisSpeed.omegaRadiansPerSecond);
    }

    public MKIFieldRelativeSpeed() {
        this.vx = 0.0;
        this.vy = 0.0;
        this.omega = 0.0;
    }
}
