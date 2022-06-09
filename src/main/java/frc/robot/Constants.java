// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.pathplanning.MKISwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Constants for swerve modules
    public static final class ModuleConstants {
        public static final double kTicksPerRotation = 2048;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.7); 
        public static final double kDriveMotorGearRatio = 1 / 6.55;                
        public static final double kTurningGearRatio = 1 / 10.29;                   
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderTicks2Meter = kDriveEncoderRot2Meter / kTicksPerRotation; // ? Wrote this to convert from ticks -> rpm -> meters, 0-2-Auto used built in RPM measurements instead. Need to test this logic 
        public static final double kTurningEncoderRot2Rad = kTurningGearRatio * 2 * Math.PI;
        public static final double kturningEncoderTicks2Rad = kTurningEncoderRot2Rad / kTicksPerRotation; // ? Similar to the drive equivilent, didn't test this yet and needs to be proven
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60; 
        public static final double kDriveEncoderTicks2MeterPerSec = (10 * 60 / kTicksPerRotation) * kDriveEncoderRPM2MeterPerSec; // ? Did this in my head. No clue if it will work. Velocity is ticks per 100ms, so multiply 
                                                                                                                                   // by 10 and by 60 to get ticks per minute, divide by ticks_per_rotation for RPM, then multiply by 
                                                                                                                                   // RPM to meters per second to get MPS.
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kTurningEncoderTicks2RadPerSec = (10 * 60 / kTicksPerRotation) * kTurningEncoderRPM2RadPerSec; // ? Similar logic to kDriveEncoderTicks2MetersPerSec, must be tested and proven
        public static final double kPTurning = .5;                                // ? Guess number, used by Zero to Autonomous so probably will be close
        public static final double kDTurning = .01;
    }
    
    // Constants relevant to the driving of the robot. All individual module constants subject to change
    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(20.25); // ? Needs to be measured and the placeholder has to be replaced
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(20.25); // ? Needs to be measured and the placeholder has to be replaced
        public static final MKISwerveDriveKinematics kDriveKinematics = new MKISwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   // + -
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),    // + +
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  // - -
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));  // - +

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.18;                   // ? Guess number, better throw something better in here eventually
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 64.89; // ? Guess number, better throw something better in here eventually
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;



        // Front Left Module
        public static final int kFrontLeftDriveMotorPort = 0;
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kFrontLeftTurningReversed = false;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.72;
        public static final boolean kFrontLeftDriveAbsoluteEncoderOffsetReversed = true;

        // Front Right Module
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final boolean kFrontRightDriveReversed = true;
        public static final boolean kFrontRightTurningReversed = false;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.93;
        public static final boolean kFrontRightDriveAbsoluteEncoderOffsetReversed = true;

        // Back Right Module
        public static final int kBackRightDriveMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 5;
        public static final boolean kBackRightDriveReversed = true;
        public static final boolean kBackRightTurningReversed = false;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 5.54;
        public static final boolean kBackRightDriveAbsoluteEncoderOffsetReversed = true;

        // Back Left Module
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kBackLeftTurningReversed = false;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.62;
        public static final boolean kBackLeftDriveAbsoluteEncoderOffsetReversed = true;

    }
    
    // Input constants
    public static final class OIConstants {
        public static final double kDeadband = .03; // ? Needs to be toyed with. Just picked this number at random
        public static final int kDriverControllerPort = 0;
        public static final int kDriverYAxis = 1; 
        public static final int kDriverXAxis = 0; 
        public static final int kDriverRotAxis = 4; 
        public static final int kDriverFieldOrientedButtonIdx = 6; // Currently right bumper
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 7;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0;//2.5;//2.5;
        public static final double kPYController = 0;//2.5;//2.5;
        public static final double kPThetaController = 0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, 
                                             kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ClimbConstants {
        public static final double arm_kP = .1;
        public static final double arm_kI = 0;
        public static final double arm_kD = 0;
        public static final double arm_kIz = 0;
        public static final double arm_kFF = 0;
        public static final double arm_maxOutput = 1;
        public static final double arm_minOutput = -1;
        public static final double arm_maxVel = 2000;
        public static final double arm_minVel = 0;
        public static final double arm_maxAcc = 1500;
        public static final double arm_allowedErr = 100;
    }
    // Pigeon 1/2 ID in Phoenix Tuner
    public static final int kPigeonCANID = 14; // ? Needs to be located. This is a number from last robot
}
