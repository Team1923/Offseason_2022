package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DesiredClimb {

    public enum Climbs{
        LEVEL_TWO,
        LEVEL_THREE,
        TRAVERSAL_ARMS_EXTENDED,
        FULL_TRAVERSAL
    }

    public Climbs climb;

    //idk these IDs we need to fix them
    public Climbs updateCurrentClimb(Joystick joystick){
        //full traversal
        if(joystick.getRawButton(8)){
            return Climbs.FULL_TRAVERSAL;
        }
        else if(joystick.getRawButton(8)){
            return Climbs.TRAVERSAL_ARMS_EXTENDED;
        }
        else if(joystick.getRawButton(8)){
            return Climbs.LEVEL_THREE;
        }
        else if(joystick.getRawButton(8)){
            return Climbs.LEVEL_TWO;
        }
        return Climbs.FULL_TRAVERSAL;
    }

}
