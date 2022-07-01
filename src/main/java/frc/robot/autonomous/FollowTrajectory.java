// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LoadTrajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.pathplanning.MKISwerveControllerCommand;
import frc.robot.pathplanning.MKITrajectoryGenerator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectory extends SequentialCommandGroup {

  // Declare swerve subsystem
  SwerveSubsystem SWERVE_SUBSYSTEM;

  LoadTrajectory trajectoryLoader;

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(SwerveSubsystem swerve, String path) {

    // Set our local swerve subsystem to be equal to the one that was passed in, so we can access all of the same motors
    //this.SWERVE_SUBSYSTEM = swerve;
    this.SWERVE_SUBSYSTEM = swerve;

    this.trajectoryLoader = new LoadTrajectory("/routines/" + path);
    
    //SmartDashboard.putString("Path ID: ", path);

    // Load a trajectory fron the specified path and convert it to a WPILib trajectory
    Trajectory loaded_trajectory = MKITrajectoryGenerator.generateTrajectory(trajectoryLoader.getInitialPose(), trajectoryLoader.getTranslations(), trajectoryLoader.getFinalPose(), trajectoryLoader.getInitialVelocity(), trajectoryLoader.getFinalVelocity());
    
    SmartDashboard.putString("Start Pose", trajectoryLoader.getInitialPose().toString());
    SmartDashboard.putString("End Pose", trajectoryLoader.getFinalPose().toString());
    // Instantiate the x and y PID controllers. They operate indepentently (Holomonic system)
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

    // This PID controller is a little bit different as we want to restrict some of the outputs. We tell 
    // it that we have a defined max angular velocity and acceleration.
    ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, 
                AutoConstants.kThetaControllerConstraints);

    // Allow the robot to realize the wheels can spin in a full circle, allowing for jumps from max and min bounds
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    System.out.println(loaded_trajectory);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> SWERVE_SUBSYSTEM.resetOdometry(loaded_trajectory.getInitialPose())),
      new MKISwerveControllerCommand(
        loaded_trajectory,
        SWERVE_SUBSYSTEM::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        SWERVE_SUBSYSTEM::setModuleStates,
        SWERVE_SUBSYSTEM
      ), 
      new InstantCommand(() -> SWERVE_SUBSYSTEM.stop()),
      new InstantCommand(() -> SWERVE_SUBSYSTEM.resetEncoders())
      );
  }






// Storing these things at the end of the file in case they are needed for the future.
// This is the manual way to make trajectories in WPILib. Trajectory config is used
// to define the rotational constraints

    // //Create trajectory settings
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     .setKinematics(DriveConstants.kDriveKinematics
    //     );

    // //Generate Trajectory
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //     trajectoryLoader.getInitialPose(),
    //     trajectoryLoader.getTranslations(),
    //     trajectoryLoader.getFinalPose(),
    //     trajectoryConfig
    //     );

}