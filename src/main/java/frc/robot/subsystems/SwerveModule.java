// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new WPI_TalonFX(driveMotorId);
        turningMotor = new WPI_TalonFX(turningMotorId);

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
        

        turningPidController = new PIDController(Constants.kPTurning, 0, 0);
    }
}
