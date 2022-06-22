package frc.robot.MKILib;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class MKIPoseEstimator {
    private Translation2d position;
    private Supplier<SwerveModuleState> topLeft, 
        topRight, 
        bottomLeft, 
        bottomRight;
    private Supplier<List<Double>> gyroSupplier;

    private float deltaTime;
    private float lastTimeMillis;

    private double biasModifier = 1;

    public MKIPoseEstimator(Supplier<SwerveModuleState> topL, Supplier<SwerveModuleState> topR, Supplier<SwerveModuleState> bottomL, 
                                                        Supplier<SwerveModuleState> bottomR, Translation2d initialPosition, Supplier<List<Double>> gyro){
        topLeft = topL;
        topRight = topR;
        bottomLeft = bottomL;
        bottomRight = bottomR;
        deltaTime = 0f;
        gyroSupplier = gyro;

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

        // Define each of the wheel velocity vectors from their respective state information
        Vector2d topLeftVector = new Vector2d(topLeft.get().speedMetersPerSecond*topLeft.get().angle.getCos(), 
            topLeft.get().speedMetersPerSecond*topLeft.get().angle.getSin());

        Vector2d topRightVector = new Vector2d(topRight.get().speedMetersPerSecond*topRight.get().angle.getCos(), 
            topRight.get().speedMetersPerSecond*topRight.get().angle.getSin());

        Vector2d bottomLeftVector = new Vector2d(bottomLeft.get().speedMetersPerSecond*bottomLeft.get().angle.getCos(), 
            bottomLeft.get().speedMetersPerSecond*bottomLeft.get().angle.getSin());

        Vector2d bottomRightVector = new Vector2d(bottomRight.get().speedMetersPerSecond*bottomRight.get().angle.getCos(), 
            bottomRight.get().speedMetersPerSecond*bottomRight.get().angle.getSin());

        // Sum the vector components
        double xSum = (topLeftVector.x + topRightVector.x + bottomLeftVector.x + bottomRightVector.x);
        double ySum = (topLeftVector.y + topRightVector.y + bottomLeftVector.y + bottomRightVector.y);

        // Magnitude of the chassis velocity as measured by the encoder 
        double encoderMagnitude = Math.sqrt((xSum * xSum) + (ySum * ySum));

        // Magnitude of the chassis velocity as measured by the gyro
        double gyroMagnitude = MKIMath.magnitude(gyroSupplier.get());

        // This will signify how different the measured values are. Subtracting one to define an equivlent measurement from the gyro and 
        // encoders to mean we do NOT use the gyro in our calculation.
        double magnitudeDifference = (encoderMagnitude / gyroMagnitude) - 1;
        
        // Calculate how much weight we should assign to the gyro data, versus the encoder data.
        double gyroBias = Math.tanh(biasModifier * magnitudeDifference);

        // Get the filtered chassis x-velocity vector magnitude
        double finalXVector = (gyroBias * gyroSupplier.get().get(0)) + ((1-gyroBias) * xSum);

        // Get the filtered chassis x-velocity vector magnitude
        double finalYVector = (gyroBias * gyroSupplier.get().get(1)) + ((1-gyroBias) * ySum);

        // Set the new position to be the old position plus our velocity magnitudes * deltaTime
        position = new Translation2d(position.getX() + (finalXVector * deltaTime), position.getY() + (finalYVector * deltaTime));
        

    }
}
