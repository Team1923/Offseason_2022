package frc.robot.MKILib;

import java.util.Vector;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class MKIPoseEstimator {
    private Translation2d position;
    private Supplier<SwerveModuleState> topLeft, 
        topRight, 
        bottomLeft, 
        bottomRight;
    
    float deltaTime;
    float lastTimeMillis;


    public MKIPoseEstimator(Supplier<SwerveModuleState> topL, Supplier<SwerveModuleState> topR, Supplier<SwerveModuleState> bottomL, Supplier<SwerveModuleState> bottomR, Translation2d initialPosition){
        topLeft = topL;
        topRight = topR;
        bottomLeft = bottomL;
        bottomRight = bottomR;
        deltaTime = 0f;
        lastTimeMillis = System.currentTimeMillis();

        position = initialPosition;
    }

    public Translation2d getPosition(){
        return position;
    }

    public void setPosition(Translation2d pos){
        position = pos;
    }

    public void updatePosition(){
        deltaTime = System.currentTimeMillis() - lastTimeMillis;
        lastTimeMillis = System.currentTimeMillis();
        deltaTime *= 1000;
        Vector2d topLeftVector = new Vector2d(topLeft.get().speedMetersPerSecond*topLeft.get().angle.getCos(), 
            topLeft.get().speedMetersPerSecond*topLeft.get().angle.getSin());

        Vector2d topRightVector = new Vector2d(topRight.get().speedMetersPerSecond*topRight.get().angle.getCos(), 
            topRight.get().speedMetersPerSecond*topRight.get().angle.getSin());

        Vector2d bottomLeftVector = new Vector2d(bottomLeft.get().speedMetersPerSecond*bottomLeft.get().angle.getCos(), 
            bottomLeft.get().speedMetersPerSecond*bottomLeft.get().angle.getSin());

        Vector2d bottomRightVector = new Vector2d(bottomRight.get().speedMetersPerSecond*bottomRight.get().angle.getCos(), 
            bottomRight.get().speedMetersPerSecond*bottomRight.get().angle.getSin());

        double xSum = (topLeftVector.x + topRightVector.x + bottomLeftVector.x + bottomRightVector.x) * deltaTime;
        double ySum = (topLeftVector.y + topRightVector.y + bottomLeftVector.y + bottomRightVector.y) * deltaTime;

        position = new Translation2d(position.getX() + xSum, position.getY() + ySum);
        

    }
}
