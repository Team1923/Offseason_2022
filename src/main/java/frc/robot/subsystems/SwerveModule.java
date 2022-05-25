// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {

    // Motors
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;

    // Turning PID controller 
    private final PIDController turningPidController;

    // Absolute encoder junk
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    // Constructor to instantiate a new Swerve Module. A bunch of inputs that looks messy but Zero to Autonomous did it 
    // and I have faith in them, so lets roll with it
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        // Absolute encoder stuff so the wheel always knows where it is
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        // Initialize both motors of the module
        driveMotor = new WPI_TalonFX(driveMotorId);
        turningMotor = new WPI_TalonFX(turningMotorId);

        // Sets each of the motors to be reversed based on their respective inputs in the constructor
        if(driveMotorReversed) {
            driveMotor.setInverted(InvertType.InvertMotorOutput);
        } else {
            driveMotor.setInverted(InvertType.None);
        }

        if(turningMotorReversed) {
            turningMotor.setInverted(InvertType.InvertMotorOutput);
        } else {
            turningMotor.setInverted(InvertType.None);
        }
        
        // Instantiating PID controller for the steering motor, I think that a kP value will be 
        // enough to get the wheel to it's heading (pending testing)
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

        // Set the PID controller to be "continious" between -PI and PI radians
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // Resets the encoders when the module is instantiated
        resetEncoders();
    }

    // ? Multiplies the raw feedback of ticks per 100ms to a constant hopefully that will convert the output from Ticks to Meters Traveled
    public double getDrivePositionMeters() {
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderTicks2Meter;
    }

    // ? Multiplies the output of the turning motor in ticks by a constant that will generate radians traveled (Ticks to Radians)
    public double getTurningPositionRads() {
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.kturningEncoderTicks2Rad;
    }

    // ? Returns hopefully a conversion of ticks per 100ms to meters per second of the drive wheel
    public double getDriveVelocityMPS() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderTicks2MeterPerSec;
    }

    // ? Returns hopefully a conversion of ticks per 100ms to radians per second of the turning wheel
    public double getTurningVelocityRPS() {
        return turningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderTicks2RadPerSec;
    }

    // ? Getter method that returns the offset absolute encoder position in radians
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    // ? Resets the encoders to their starting position, turning motor has to be put to the position that the absolute encoder measures.
    // This is done with some wonky conversions from the offset radians -> rotations of turning gear -> rotations of motor shaft -> ticks. Needs to be tested
    // and verified that the math and logic works.
    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        double radians_from_ticks = ((getAbsoluteEncoderRad() / (2 * Math.PI)) / ModuleConstants.kTurningGearRatio) * ModuleConstants.kTicksPerRotation;
        turningMotor.setSelectedSensorPosition(radians_from_ticks);
    }

    // ? A way to visualize the current state of the module. WPILib requires this format for a lot of the actual swerve drive code.
    // Might be able to use it to print out a graph of each wheel on Shuffleboard? Requires investigation
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMPS(), new Rotation2d(getTurningPositionRads()));
    }

    // Update the goal of the swerve module
    public void setDesiredState(SwerveModuleState state) {

        // If there is no substantial change in velocity from current state to new state, do not change anything.
        // This just negates the fact that the motors are going to return to zero if you let go of the stick, which is very annoying when driving
        if(Math.abs(state.speedMetersPerSecond) < .001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        
        // ? Using percent output right now, but maybe we should do closed-loop velocity control and just multiply this fraction by 
        // the max RPM of a falcon motor. Might provide smoother results. Anyhow, percent output is a good start
        driveMotor.set(ControlMode.PercentOutput, (state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));

        // ? Similar to above, might be better to do closed-loop velocity control and multiply this output by the max RPM of a falcon
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPositionRads(), state.angle.getRadians()));
        SmartDashboard.putString(Integer.toString(absoluteEncoder.getChannel()), Double.toString(getTurningPositionRads()));

        SmartDashboard.putString("PID OUTPUT " + absoluteEncoder.getChannel() + ":", Double.toString(turningPidController.calculate(getTurningPositionRads(), state.angle.getRadians())));
        // Recommended debug printout for swerve state
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "]", state.toString());
    }

    // Stop function 
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
