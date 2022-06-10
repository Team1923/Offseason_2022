package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveyorSubsystem. */
  private WPI_TalonFX conveyorMotor = new WPI_TalonFX(ConveyorConstants.conveyorMotorID);

  private DigitalInput beamBreakOne;
  private DigitalInput beamBreakTwo;

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

    beamBreakOne = new DigitalInput(1);
    beamBreakTwo = new DigitalInput(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("BEAM BREAK ONE: ", beamBreakOne.get());
    SmartDashboard.putBoolean("BEAM BREAK TWO: ", beamBreakTwo.get());
  }

  public void setConveyor(double percentOut){
    conveyorMotor.set(ControlMode.PercentOutput, percentOut);
  }

  public void stopConveyor(){
    conveyorMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getFrontBeamBreak() {
    return beamBreakOne.get();
  }

  public boolean getBackBeamBreak() {
    return beamBreakOne.get();
  }

}
