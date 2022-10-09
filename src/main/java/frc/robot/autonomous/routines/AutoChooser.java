package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.StateHandler;

public class AutoChooser {
    public enum AutoMode{
        ONE_BALL,
        ONE_BALL_GET_OUT,
        TWO_BALL,
        GOON_BALL,
        FOUR_BALL,
        THREE_BALL,
        THREE_BALL_ONE_GOON, 
        FIVE_BALL,
        TEST_PATH
    }

    private SendableChooser<AutoMode> chooser;
    ShuffleboardTab driverDashboard = Shuffleboard.getTab("Coach Dashboard");
    ShuffleboardLayout auto = driverDashboard.getLayout("Auto Setup", "List Layout").withPosition(6, 0).withSize(2, 2);
    
    public AutoChooser(){
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("ONE BALL", AutoMode.ONE_BALL);
        chooser.addOption("ONE BALL GET OUT", AutoMode.ONE_BALL_GET_OUT);
        chooser.addOption("TWO BALL", AutoMode.TWO_BALL);
        chooser.addOption("GOON BALL", AutoMode.GOON_BALL);
        chooser.addOption("FOUR BALL", AutoMode.FOUR_BALL);
        chooser.addOption("THREE BALL", AutoMode.THREE_BALL);
        chooser.addOption("FIVE BALL", AutoMode.FIVE_BALL);
        chooser.addOption("TEST PATH", AutoMode.TEST_PATH);
        auto.add(chooser);
    }

    public Command startMode(
        SwerveSubsystem swerve, 
        ShooterSubsystem shooter, 
        ConveyorSubsystem conveyor, 
        IntakeSubsystem intake, 
        HoodSubsystem hood, 
        LimelightSubsystem limelight,
        StateHandler stateHandler
    ){
        AutoMode mode = (AutoMode)(chooser.getSelected());
        switch(mode){
            case ONE_BALL:
                return new UnoBall(swerve, shooter, conveyor, intake, hood, stateHandler);
            case ONE_BALL_GET_OUT:
                return new OneBallGetOut(swerve, shooter, conveyor, intake, hood);
            case TWO_BALL:
                return new DeuxBall(swerve, shooter, conveyor, intake, hood, limelight, stateHandler);
            case GOON_BALL:
                return new GoonBall(swerve, shooter, conveyor, intake, hood, limelight, stateHandler);
            case FOUR_BALL:
                return new SiBall(swerve, shooter, conveyor, intake, hood, limelight);
            case THREE_BALL:
                return new ThreeBall(swerve, shooter, conveyor, intake, hood, limelight, stateHandler);
            case FIVE_BALL:
                return new FunfBall(swerve, hood, conveyor, intake, shooter, limelight, stateHandler);
            case TEST_PATH:
                return new PathPlannerTestSequence(swerve);
            default:
                return new DeuxBall(swerve, shooter, conveyor, intake, hood, limelight, stateHandler);
        }
    }
}
