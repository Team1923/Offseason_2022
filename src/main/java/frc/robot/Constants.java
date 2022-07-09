// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
        public static final double kDriveMotorGearRatio = 1.0 / 6.55;                
        public static final double kTurningGearRatio = 1.0 / 10.29;                   
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
        public static final double kPTurning = 0.35;                                // ? Guess number, used by Zero to Autonomous so probably will be close
        public static final double kDTurning = 0.0;
    }
    
    // Constants relevant to the driving of the robot. All individual module constants subject to change
    public static final class DriveConstants {
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(20.25); // ? Needs to be measured and the placeholder has to be replaced
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(20.25); // ? Needs to be measured and the placeholder has to be replaced
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
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
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.71;
        public static final boolean kFrontLeftDriveAbsoluteEncoderOffsetReversed = true;

        // Front Right Module
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final boolean kFrontRightDriveReversed = true;
        public static final boolean kFrontRightTurningReversed = false;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.90;
        public static final boolean kFrontRightDriveAbsoluteEncoderOffsetReversed = true;

        // Back Right Module
        public static final int kBackRightDriveMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 5;
        public static final boolean kBackRightDriveReversed = false;
        public static final boolean kBackRightTurningReversed = false;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.44;
        public static final boolean kBackRightDriveAbsoluteEncoderOffsetReversed = true;

        // Back Left Module
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kBackLeftTurningReversed = false;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.62;//5.97;//3.62;
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
        public static final int kDriverAButton = 1;
        public static final int kDriverBButton = 2;
        public static final int kDriverXButton = 3;
        public static final int kDriverYButton = 4;

        public static final int kOperatorControllerPort = 1;
        public static final int kOperatorLeftYAxis = 1;
        public static final int kOperatorLeftXAxis = 0;
        public static final int kOperatorRightYAxis = 5;
        public static final int kOperatorRightXAxis = 4;
        public static final int kOperatorXButton = 1;
        public static final int kOperatorCircleButton = 2;
        public static final int kOperatorSquareButton = 3;
        public static final int kOperatorTriangleButton = 4;
        public static final int kOperatorLeftBumper = 5;
        public static final int kOperatorRightBumper = 6;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 7;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.2;//2.5;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 10;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, 
                                             kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ClimbConstants {
        public static final double arm_kP = .3;
        public static final double arm_kI = 0;
        public static final double arm_kD = 0;
        public static final double arm_kF = 0;;
    }

    
    public static final class IntakeConstants {
        public static final int leftIntakeMotorID = 8; //will need to be changed
        public static final int rightIntakemotorID = 9; //will need to be changed
        public static final double intakePercentOut = 0.5; //change this
        public static final double intakeOutPercentOut = -0.9;
        public static final int currentLimit = 30; //still needs to be set
        public static final int thresholdLimit = 40; //still needs to be set I just guessed
        public static final SupplyCurrentLimitConfiguration intakeCurrentLimit =
            new SupplyCurrentLimitConfiguration(true, currentLimit, thresholdLimit, 0.2);
    }
    
    public static final class ConveyorConstants {
        public static final int conveyorMotorID = 10; //will need to be changed
        public static final double conveyorPercentOut = 0.5; //change this
        public static final double conveyorOutPercentOut = -0.9;
        public static final double conveyorShootPercentOut = 0.96731; //change this
        public static final double cancelStallPercentOut = -.15; //change this
        public static final int currentLimit = 30; //still needs to be set
        public static final int thresholdLimit = 40; //still needs to be set I just guessed
        public static final SupplyCurrentLimitConfiguration conveyorCurrentLimit =
            new SupplyCurrentLimitConfiguration(true, currentLimit, thresholdLimit, 0.2);
    }

    public static final class ShooterConstants {
        public static final int leftShooterMotorID = 11; //change
        public static final int rightShooterMotorID = 12; //change
        public static final int sCurrentLimit = 30; //change
        public static final int sThresholdLimit = 40;
        public static final SupplyCurrentLimitConfiguration shooterCurrentLimit = 
            new SupplyCurrentLimitConfiguration(true, sCurrentLimit, sThresholdLimit, 0.2);
        
        // Tune these to achieve accurate shooter wheel startup 
        public static final double shooterkP = .2;
        public static final double shooterkI = 0;
        public static final double shooterkD = 0.4;
        public static final double shooterkFF = 0.05;

        // Shooter target RPM threshold
        public static final double shooterRPMThreshold = 150;
        public static final double shooterTimeThreshold = .1;

        public static final double avoidStallSpeed = 0.1;

        public static final double shooterPercentOut = 0.25;
        
    }

    public static final class HoodConstants {
        public static final int hoodMotorID = 13; //change

        public static final double zeroPosition = 0;

        public static final int hCurrentLimit = 30;
        public static final int hThresholdLimit = 40;
        public static final SupplyCurrentLimitConfiguration hoodCurrentLimit = 
            new SupplyCurrentLimitConfiguration(true, hCurrentLimit, hThresholdLimit, 0.2);
        
        public static final double hood_shootkP = 1;
        public static final double hood_shootkI = 0;
        public static final double hood_shootkD = 0;
        public static final double hood_shootkFF = 0;

        public static final double hood_climbkP = .3;
        public static final double hood_climbkI = 0;
        public static final double hood_climbkD = 0;
        public static final double hood_climbkFF = 0;

        public static final double hoodMinPosition = 0;
        public static final double hoodMaxPosition = 39835;
        public static final double hoodScaleFactor = 1; // used to adjust the overall movement of the hood. Adjusts the angle of
                                                        // hood by this scale factor. Code should still never let it go above the max, though.
    }
    
    public static final class LimelightConstants {
        public static final double limelightMountingHeight = 17.57; // inches from ground to center
        public static final double LimelightMountingAngle = 45; // angle from center of camera above horizontal plane
        public static final double centerGoalHeight = 103; // inches from ground to center

    }
    // Pigeon 1/2 ID in Phoenix Tuner
    public static final int kPigeonCANID = 14;

    public static final double ticksPerRev = 2048;

    public static final int timeoutMs = 20;

    public static final double falconMaxRPM = 6380;

    public static final double gyroToMPS = 1;

    public static final Point hubPosition = new Point(8.3, 4.2); // NEEDS TO BE UPDATED,,, BUT MAKING Y POSITIVE. HAVE TO INVERT ODOMETRY INPUT.

}
