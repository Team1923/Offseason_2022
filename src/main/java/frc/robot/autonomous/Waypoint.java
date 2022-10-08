package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Waypoint {
    private final Translation2d anchorPoint;
    private final Translation2d prevControl;
    private final Translation2d nextControl;
    private final double velOverride;
    private final Rotation2d holonomicRotation;
    protected final boolean isReversal;

    public Waypoint(Translation2d anchorPoint, Translation2d prevControl, Translation2d nextControl, double velOverride, Rotation2d holonomicRotation, boolean isReversal){
        this.anchorPoint = anchorPoint;
        this.prevControl = prevControl;
        this.nextControl = nextControl;
        this.velOverride = velOverride;
        this.holonomicRotation = holonomicRotation;
        this.isReversal = isReversal;
    }
}