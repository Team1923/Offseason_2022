// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private Solenoid solenoid3 = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  private Solenoid solenoid4 = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
  



  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    setLow();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setHigh() {
    solenoid1.set(true);
    solenoid2.set(false);
    solenoid3.set(false);
    solenoid4.set(true);
  }

  public void setLow() {
    solenoid1.set(false);
    solenoid2.set(true);
    solenoid3.set(true);
    solenoid4.set(false);
  }
}
