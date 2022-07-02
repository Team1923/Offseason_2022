package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.LoadTrajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.pathplanning.MKITrajectoryGenerator;
import frc.robot.subsystems.SwerveSubsystem;

public class RunTrajectory extends SequentialCommandGroup {

    private LoadTrajectory trajectoryLoader;

    public RunTrajectory(SwerveSubsystem s_Swerve, String path){
        trajectoryLoader = new LoadTrajectory("/routines/" + path + ".csv");
        
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);
        config.setStartVelocity(trajectoryLoader.getInitialVelocity());
        config.setEndVelocity(trajectoryLoader.getFinalVelocity());
        config.setReversed(true);
        

       // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(1, 1)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(2, 0, new Rotation2d(Math.PI/2)),
        //         config);

        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                trajectoryLoader.getInitialPose(),
                trajectoryLoader.getTranslations(),
                trajectoryLoader.getFinalPose(),
                config
            );

            System.out.println(exampleTrajectory.toString());


        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        
        addCommands(
            new InstantCommand(() -> SmartDashboard.putString("Is Running", "e")),
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}