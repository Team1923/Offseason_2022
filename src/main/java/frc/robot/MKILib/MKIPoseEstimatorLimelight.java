package frc.robot.MKILib;

import java.util.List;
import java.util.function.Supplier;



import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.interfaces.LimelightInterface;

public class MKIPoseEstimatorLimelight extends MKIPoseEstimator{

    private LimelightInterface limelight;
    

    public MKIPoseEstimatorLimelight(Supplier<SwerveModuleState> topL, Supplier<SwerveModuleState> topR, Supplier<SwerveModuleState> bottomL, Supplier<SwerveModuleState> bottomR, 
        Translation2d initialPosition, LimelightInterface limelightInterface, Supplier<List<Double>> gyroSupplier){
        super(topL, topR, bottomL, bottomR, initialPosition, gyroSupplier);
        limelight = limelightInterface;
    }

    public void fixPoseLimelight(){
        // Might want to add another condition here... like limelight has a target & it has a certain size bounding box to avoid random errors.
        if(limelight.validTargets()) {
            double distanceToGoal = limelight.distanceToTarget();
            Point robotPosition = new Point(super.getPosition().getX(), super.getPosition().getY());
            Point intersectionPoint = MKIMath.calculateIntersection(Constants.hubPosition, robotPosition, Constants.hubPosition, distanceToGoal).get(0);
            Translation2d newPosition = new Translation2d(intersectionPoint.x, intersectionPoint.y);
            super.setPosition(newPosition);
        } 
        else{
            return;
        }
    }

}
