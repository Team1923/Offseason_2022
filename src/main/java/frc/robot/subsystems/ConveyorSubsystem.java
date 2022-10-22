package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveyorSubsystem. */
  public WPI_TalonFX conveyorMotor = new WPI_TalonFX(ConveyorConstants.conveyorMotorID);

  private DigitalInput beamBreakOne;
  private DigitalInput beamBreakTwo;

  // ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");

  // ShuffleboardLayout conveyorLayout = coachTab.getLayout("Conveyor", "List Layout").withPosition(1, 0).withSize(1, 3);

  // private NetworkTableEntry beamBreak1 = conveyorLayout.add("Beam Break 1", false)
  // .withSize(1, 1)
  // .withPosition(0, 4)
  // .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
  // .getEntry();

  // private NetworkTableEntry beamBreak2 = conveyorLayout.add("Beam Break 2", false)
  // .withSize(1, 1)
  // .withPosition(0, 4)
  // .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
  // .getEntry();

  // private NetworkTableEntry isConveyorRunning = conveyorLayout.add("Conveyor Running", false)
  // .withSize(1, 1)
  // .withPosition(0, 4)
  // .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
  // .getEntry();
  
  public ConveyorSubsystem() { 
    
    //configure factory default
    conveyorMotor.configFactoryDefault();

    //brake mode
    conveyorMotor.setNeutralMode(NeutralMode.Brake);

    //current limit
    conveyorMotor.configSupplyCurrentLimit(ConveyorConstants.conveyorCurrentLimit);

    //configure nominal output
    conveyorMotor.configNominalOutputForward(1.0, Constants.timeoutMs); //will need to set these properly, constants are just a guess
    conveyorMotor.configNominalOutputReverse(-1.0, Constants.timeoutMs);

    //configure peak output
    conveyorMotor.configPeakOutputForward(1.0, Constants.timeoutMs);
    conveyorMotor.configPeakOutputReverse(-1.0, Constants.timeoutMs);

    beamBreakOne = new DigitalInput(0);
    beamBreakTwo = new DigitalInput(1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboar.putBoolean("BEAM BREAK ONE: ", getFrontBeamBreak());
   // SmartDashboar.putBoolean("BEAM BREAK TWO: ", getBackBeamBreak());

   // SmartDashboar.putNumber("Running Conveyor?", conveyorMotor.get());

    // isConveyorRunning.setBoolean(Math.abs(conveyorMotor.get()) > 0);
    //SmartDashboard.putNumber("System Time", System.currentTimeMillis());
  }

  public void setConveyor(double percentOut){
    conveyorMotor.set(ControlMode.PercentOutput, percentOut);
  }

  public void stop(){
    conveyorMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getFrontBeamBreak() {
    // beamBreak1.setBoolean(!beamBreakOne.get());
    return !beamBreakOne.get();
  }

  public boolean getBackBeamBreak() {
    // beamBreak2.setBoolean(!beamBreakTwo.get());
    return !beamBreakTwo.get();
  }

}
