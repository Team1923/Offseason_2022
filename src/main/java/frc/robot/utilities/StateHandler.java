// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.OIConstants;
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

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

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

    ShuffleboardLayout stateLayout = coachTab.getLayout("State", "List Layout").withPosition(0, 2).withSize(1, 1);

    ShuffleboardLayout twoBallState = coachTab.getLayout("Two Balls?", "List Layout").withPosition(4,0).withSize(3, 2);

    NetworkTableEntry currentState = stateLayout.add("State", States.NO_BALLS.toString()).getEntry();

    private NetworkTableEntry twoBalls = twoBallState.add("i n t e r e s t i n g", false)
  .withSize(3, 3)
  .withPosition(4, 0)
  .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
  .getEntry();
    

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
                    twoBalls.setBoolean(false);
                    stopRumble();
                }

                if(this.backBeamBreak) {
                    currentRobotState = States.ONE_BALL_FAR_BROKEN;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }
                break;
            case ONE_BALL_CLOSE_BROKEN:
                if(!this.frontBeamBreak && !intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }

                if(!this.frontBeamBreak && intake_reverse) {
                    this.currentRobotState = States.NO_BALLS;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }

                if(!this.frontBeamBreak && this.backBeamBreak) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }
                break;
            case ONE_BALL_NONE_BROKEN:
                if(this.frontBeamBreak && intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }

                if(this.backBeamBreak && !intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }
                break;
            case ONE_BALL_FAR_BROKEN:
                if(!this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.NO_BALLS;
                    speaker.playSound(1);
                    twoBalls.setBoolean(false);
                    stopRumble();
                }

                if(!this.backBeamBreak && this.intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }
                if(!this.backBeamBreak && !this.intake_reverse){
                    this.currentRobotState = States.NO_BALLS;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }
                break;
            case TWO_BALLS_BOTH_BROKEN:
                if(!this.frontBeamBreak && this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.ONE_BALL_FAR_BROKEN;
                    speaker.playSound(1);
                    twoBalls.setBoolean(false);
                    stopRumble();
                }

                if(!this.frontBeamBreak && !this.backBeamBreak && this.acceptableRPM) {
                    this.currentRobotState = States.ONE_BALL_NONE_BROKEN;
                    speaker.playSound(1);
                    twoBalls.setBoolean(false);
                    stopRumble();
                }

                if(this.frontBeamBreak && !this.backBeamBreak && intake_reverse) {
                    this.currentRobotState = States.ONE_BALL_CLOSE_BROKEN;
                    twoBalls.setBoolean(false);
                    stopRumble();
                }
                break;
            default:
                break;
        }

        if(this.frontBeamBreak && this.backBeamBreak) {
            currentRobotState = States.TWO_BALLS_BOTH_BROKEN;
            rumbleControllers();
            twoBalls.setBoolean(true);
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

    public void rumbleControllers(){
        driverJoystick.setRumble(RumbleType.kLeftRumble, 1);
        driverJoystick.setRumble(RumbleType.kRightRumble, 1);
        operatorJoystick.setRumble(RumbleType.kLeftRumble, 1);
        operatorJoystick.setRumble(RumbleType.kRightRumble, 1);
    }

    public void stopRumble(){
        driverJoystick.setRumble(RumbleType.kRightRumble, 0);
        driverJoystick.setRumble(RumbleType.kLeftRumble, 0);
        operatorJoystick.setRumble(RumbleType.kRightRumble, 0);
        operatorJoystick.setRumble(RumbleType.kLeftRumble, 0);
    }

}
