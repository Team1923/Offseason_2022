package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveyorSubsystem. */
  private WPI_TalonFX conveyorMotor = new WPI_TalonFX(ConveyorConstants.conveyorMotorID);

  public ConveyorSubsystem() {
    //configure factory default
    conveyorMotor.configFactoryDefault();

    //brake mode
    conveyorMotor.setNeutralMode(NeutralMode.Brake);

    //current limit
    conveyorMotor.configSupplyCurrentLimit(ConveyorConstants.conveyorCurrentLimit);

    //configure nominal output
    conveyorMotor.configNominalOutputForward(0, 30); //will need to set these properly, constants are just a guess
    conveyorMotor.configNominalOutputReverse(0, 30);

    //configure peak output
    conveyorMotor.configPeakOutputForward(1, 30);
    conveyorMotor.configPeakOutputReverse(-1, 30);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setConveyor(double percentOut){
    conveyorMotor.set(ControlMode.PercentOutput, percentOut);
  }

  public void stopConveyor(){
    conveyorMotor.set(ControlMode.PercentOutput, 0);
  }
}
