package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DesiredClimb {

    public enum Climbs{
        LEVEL_TWO,
        LEVEL_THREE,
        TRAVERSAL_ARMS_EXTENDED,
        FULL_TRAVERSAL
    }

    private Climbs climb = Climbs.FULL_TRAVERSAL;



    //idk these IDs we need to fix them
    public void updateCurrentClimb(Joystick joystick){

        SmartDashboard.putNumber("Operator DPAD Value: ", joystick.getPOV());

        

        //full traversal
        if(joystick.getPOV() == 0){
            climb = Climbs.FULL_TRAVERSAL;
        }
        else if(joystick.getPOV() == 90){
            climb = Climbs.TRAVERSAL_ARMS_EXTENDED;
        }
        else if(joystick.getPOV() == 270){
            climb = Climbs.LEVEL_THREE;
        }
        else if(joystick.getPOV() == 180){
            climb = Climbs.LEVEL_TWO;
        }

    }

    public Climbs getCurrentClimb(){
        return climb;
    }

}
