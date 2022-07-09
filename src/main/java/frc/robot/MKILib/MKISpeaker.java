// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MKILib;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class MKISpeaker {

    private DigitalOutput sound_one;
    private DigitalOutput sound_two;
    private DigitalOutput sound_three;
    private DigitalOutput sound_four;

    public MKISpeaker() {
        sound_one = new DigitalOutput(7);
        sound_two = new DigitalOutput(8);
        sound_three = new DigitalOutput(9);
        sound_four = new DigitalOutput(10);

        sound_one.set(false);
        sound_two.set(false);
        sound_three.set(false);
        sound_four.set(false);
    }

    // Send a 10ms 5v pulse to the digital output 
    public void playSound(int id) {
        System.out.println("TEST 1");
        switch(id) {
            case 1:
            System.out.println("TEST 2");
                new Thread(() -> {
                    try {
                        System.out.println("TEST 3");
                        sound_one.set(true);
                        Thread.sleep(200);
                        sound_one.set(false);
                    } catch (Exception e) {}
                  }).start();
                break;
            case 2:
                new Thread(() -> {
                    try {
                        sound_two.set(true);
                        Thread.sleep(200);
                        sound_two.set(false);
                    } catch (Exception e) {}
                }).start();
                break;
            case 3:
                new Thread(() -> {
                    try {
                        sound_three.set(true);
                        Thread.sleep(20);
                        sound_three.set(false);
                    } catch (Exception e) {}
                }).start();
                break;
            case 4:
                new Thread(() -> {
                    try {
                        sound_four.set(true);
                        Thread.sleep(20);
                        sound_four.set(false);
                    } catch (Exception e) {}
                }).start();
                break;
            default:
                DriverStation.reportError("Non-existent song ID.", true);
                break;
        }
    }

    // Print out the values of each digital output. Used for debug purposes.
    public void printTriggers() {
        SmartDashboard.putBoolean("Track 1: ", sound_one.get());
        SmartDashboard.putBoolean("Track 2: ", sound_two.get());
        SmartDashboard.putBoolean("Track 3: ", sound_three.get());
        SmartDashboard.putBoolean("Track 4: ", sound_four.get());
    }
}
