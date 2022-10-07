// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MKILib;

/** Add your docs here. */
public class MKIMovingShooter {

    // Will perform calculations to find the angle offset to make a shot as we translate a certain velocity in the X direction, if
    // we call the direction to the hub north.
    public double calculateRotationOffset(double vx, double vy, double distance_to_goal, double angle_to_goal) {

        double hub_relative_x = vx*(Math.cos(Math.toRadians(angle_to_goal))) - vy*(Math.sin(Math.toRadians(angle_to_goal)));
        double ball_velocity = 3; // Guess until we can generate a function
        

        return 1;
    }

}
