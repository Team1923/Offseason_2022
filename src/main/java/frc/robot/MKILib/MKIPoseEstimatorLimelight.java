package frc.robot.MKILib;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;



import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.interfaces.LimelightInterface;

public class MKIPoseEstimatorLimelight extends MKIPoseEstimator{

    private LimelightInterface limelight;
    

    public MKIPoseEstimatorLimelight(Supplier<SwerveModuleState> topL, Supplier<SwerveModuleState> topR, Supplier<SwerveModuleState> bottomL, Supplier<SwerveModuleState> bottomR, 
        Translation2d initialPosition, LimelightInterface limelightInterface){
        super(topL, topR, bottomL, bottomR, initialPosition);
        limelight = limelightInterface;
    }

    public void fixPoseThing(){
        if(limelight.validTargets()){
            double distanceToGoal = limelight.distanceToTarget();
            Point robotPosition = new Point(super.getPosition().getX(), super.getPosition().getY());
            Point intersectionPoint = calculateIntersection(Constants.hubPosition, robotPosition, Constants.hubPosition, distanceToGoal).get(0);
            Translation2d newPosition = new Translation2d(intersectionPoint.x, intersectionPoint.y);
            super.setPosition(newPosition);
        } 
        else{
            return;
        }
    }

    public static List<Point> calculateIntersection(Point pointA,
            Point pointB, Point center, double radius) {
        double baX = pointB.x - pointA.x;
        double baY = pointB.y - pointA.y;
        double caX = center.x - pointA.x;
        double caY = center.y - pointA.y;

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return Collections.emptyList();
        }
        // if disc == 0 ... dealt with later
        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        Point p1 = new Point(pointA.x - baX * abScalingFactor1, pointA.y
                - baY * abScalingFactor1);
        if (disc == 0) { // abScalingFactor1 == abScalingFactor2
            return Collections.singletonList(p1);
        }
        Point p2 = new Point(pointA.x - baX * abScalingFactor2, pointA.y
                - baY * abScalingFactor2);
        return Arrays.asList(p1, p2);
    }

}
