// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MKILib.MKISpeaker;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class StateHandler {

    public States currentRobotState;

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
        TWO_BALLS_BOTH_BROKEN,
        CLIMBING
    }

    private ShooterSubsystem SHOOTER_SUBSYSTEM;
    private IntakeSubsystem INTAKE_SUBSYSTEM;
    private ConveyorSubsystem CONVEYOR_SUBSYSTEM;
    private MKISpeaker speaker;

    ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");

    ShuffleboardLayout stateLayout = coachTab.getLayout("State", "List Layout").withPosition(2, 0).withSize(1, 1);

    NetworkTableEntry currentState = stateLayout.add("State", States.NO_BALLS.toString()).getEntry();
    

    // Define all of the variables required to track the state of the robot.
    public StateHandler(ShooterSubsystem shooter, IntakeSubsystem intake, ConveyorSubsystem conveyor, MKISpeaker s) {
        this.currentRobotState = States.NO_BALLS;

        this.SHOOTER_SUBSYSTEM = shooter;
        this.INTAKE_SUBSYSTEM = intake;
        this.CONVEYOR_SUBSYSTEM = conveyor;

        this.speaker = s;

        this.acceptableRPM = false;
        this.intake_reverse = false;
    }



    // Run the logic we determine to figure out the current state of the robot. 
    public void updateStates() {
        switch(currentRobotState) {
            case NO_BALLS:
                if(this.frontBeamBreak) {
                    currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                }

                if(this.backBeamBreak) {
                    currentRobotState = States.ONE_BALL_FAR_BROKEN;
                }
                break;
            case ONE_BALL_CLOSE_BROKEN:
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
                if(this.frontBeamBreak && intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                }

                if(this.backBeamBreak && !intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                }
                break;
            case ONE_BALL_FAR_BROKEN:
                if(!this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.NO_BALLS;
                    speaker.playSound(1);
                }

                if(!this.backBeamBreak && this.intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                }
                if(!this.backBeamBreak && !this.intake_reverse){
                    this.currentRobotState = States.NO_BALLS;
                }
                break;
            case TWO_BALLS_BOTH_BROKEN:
                if(!this.frontBeamBreak && this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                    speaker.playSound(1);
                }

                if(!this.frontBeamBreak && !this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                    speaker.playSound(1);
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

       // SmartDashboar.putString("Current Robot State: ", currentRobotState.toString());

        currentState.setString(currentRobotState.toString());

        speaker.printTriggers();
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

    public void resetState(){
        currentRobotState = States.NO_BALLS;
    }

}
