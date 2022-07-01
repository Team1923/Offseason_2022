// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class StateHandler {

    public States currentRobotState;
    
    public List<Balls> ballTracker = new ArrayList<>();

    public boolean frontBeamBreak;
    public boolean backBeamBreak;
    public boolean ballPastBreak;
    public boolean acceptableRPM;
    public boolean intake_reverse;

    // List of all possible states
    public enum States {
        NO_BALLS,
        ONE_BALL_CLOSE_BROKEN,
        ONE_BALL_NONE_BROKEN,
        ONE_BALL_FAR_BROKEN,
        TWO_BALLS_ONE_BROKEN,
        TWO_BALLS_BOTH_BROKEN,
        CLIMBING
    }

    public enum Balls {
        RED,
        BLUE, 
        EMPTY
    }

    private ShooterSubsystem SHOOTER_SUBSYSTEM;
    private IntakeSubsystem INTAKE_SUBSYSTEM;
    private ConveyorSubsystem CONVEYOR_SUBSYSTEM;

    private MKIPicoColorSensor colorSensor;

    // Define all of the variables required to track the state of the robot.
    public StateHandler(ShooterSubsystem shooter, IntakeSubsystem intake, ConveyorSubsystem conveyor, MKIPicoColorSensor cs) {
        this.currentRobotState = States.NO_BALLS;

        this.SHOOTER_SUBSYSTEM = shooter;
        this.INTAKE_SUBSYSTEM = intake;
        this.CONVEYOR_SUBSYSTEM = conveyor;

        this.acceptableRPM = false;
        this.intake_reverse = false;

        this.colorSensor = cs;

    }

    // Run the logic we determine to figure out the current state of the robot. 
    public void updateStates() {

        SmartDashboard.putNumber("SENSOR 1 RED", colorSensor.getRawColor(0).red);
        SmartDashboard.putNumber("SENSOR 1 BLUE", colorSensor.getRawColor(0).blue);
        SmartDashboard.putNumber("SENSOR 1 GREEN", colorSensor.getRawColor(0).green);
        SmartDashboard.putString("Ball 1 Value", colorSensor.getBallColor(1).toString());
        SmartDashboard.putString("Ball 2 Value", colorSensor.getBallColor(0).toString());
        SmartDashboard.putNumber("SENSOR 2 RED", colorSensor.getRawColor(1).red);
        SmartDashboard.putNumber("SENSOR 2 BLUE", colorSensor.getRawColor(1).blue);
        SmartDashboard.putNumber("SENSOR 2 GREEN", colorSensor.getRawColor(1).green);

        int emptyCount = Collections.frequency(ballTracker, Balls.EMPTY);

        switch(currentRobotState) {
            case NO_BALLS:

                ballTracker = Arrays.asList(Balls.EMPTY, Balls.EMPTY);

                if(this.frontBeamBreak) {
                    currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                }

                if(this.backBeamBreak) {
                    currentRobotState = States.ONE_BALL_FAR_BROKEN;
                }
                break;

            case ONE_BALL_CLOSE_BROKEN:
                if(emptyCount == 2 || (emptyCount == 0 && !intake_reverse)) {
                    ballTracker = Arrays.asList(Balls.EMPTY, colorSensor.getBallColor(1));
                } else if((emptyCount == 0) && intake_reverse) {
                    ballTracker = Arrays.asList(Balls.EMPTY, ballTracker.get(0));
                } else if (emptyCount == 1 && ballTracker.get(1) == Balls.EMPTY) {
                    ballTracker = Arrays.asList(Balls.EMPTY, ballTracker.get(0));
                }

                if(!this.frontBeamBreak && !intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                }

                if(!this.frontBeamBreak && intake_reverse) {
                    this.currentRobotState = States.NO_BALLS;
                }

                if(!this.frontBeamBreak && this.backBeamBreak) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                }
                break;
            case ONE_BALL_NONE_BROKEN:
                if (emptyCount == 1 && ballTracker.get(0) == Balls.EMPTY) {
                    ballTracker = Arrays.asList(ballTracker.get(1), Balls.EMPTY);
                }
                if(this.frontBeamBreak && intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                }

                if(this.backBeamBreak && !intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                }
                if(this.frontBeamBreak && !intake_reverse) {
                    this.currentRobotState = States.TWO_BALLS_ONE_BROKEN;
                }
                break;
            case ONE_BALL_FAR_BROKEN:

                if(emptyCount == 2) {
                    ballTracker = Arrays.asList(colorSensor.getBallColor(0), Balls.EMPTY);
                } else if (emptyCount == 1 && ballTracker.get(0) == Balls.EMPTY) {
                    ballTracker = Arrays.asList(ballTracker.get(1), Balls.EMPTY);
                }

                if(!this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.NO_BALLS;
                }

                if(!this.backBeamBreak && this.intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                }
                if(!this.backBeamBreak && !this.intake_reverse){
                    this.currentRobotState = States.NO_BALLS;
                }
                break;
            case TWO_BALLS_ONE_BROKEN:

                if(emptyCount == 1) {
                    ballTracker = Arrays.asList(ballTracker.get(1), colorSensor.getBallColor(1));
                }

                if(!this.backBeamBreak && !this.frontBeamBreak && intake_reverse) {
                    this.currentRobotState = States.NO_BALLS;
                }
                if(!this.backBeamBreak && this.frontBeamBreak && intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                }
                break;
            case TWO_BALLS_BOTH_BROKEN:

                
                ballTracker = Arrays.asList(colorSensor.getBallColor(0), colorSensor.getBallColor(1));

                if(!this.frontBeamBreak && this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                }

                if(!this.frontBeamBreak && !this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                }

                if(this.frontBeamBreak && !this.backBeamBreak && intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                }
                break;
            default:
                break;
        }

        if(this.frontBeamBreak && this.backBeamBreak) {
            currentRobotState = States.TWO_BALLS_BOTH_BROKEN;
        }

        SmartDashboard.putString("Current Robot State: ", currentRobotState.toString());
        SmartDashboard.putString("Ball Tracker: ", ballTracker.get(0) + "  " + ballTracker.get(1));
    }

    public void updateBooleans() {
        this.frontBeamBreak = CONVEYOR_SUBSYSTEM.getFrontBeamBreak();
        this.backBeamBreak = CONVEYOR_SUBSYSTEM.getBackBeamBreak();
        this.intake_reverse = INTAKE_SUBSYSTEM.getIntakeCommandedDirection();
        this.acceptableRPM = SHOOTER_SUBSYSTEM.getAcceptableRPMState();
    }

    public States getState() {
        return this.currentRobotState;
    }

    public boolean getEjecting() {
        return this.intake_reverse;
    }

    public List<Balls> getBallTracker() {
        return ballTracker;
    }

    public void resetState(){
        currentRobotState = States.NO_BALLS;
    }

}
