// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.ArrayList;
import java.util.Collection;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PlayMusic extends CommandBase {
  private Orchestra orchestra;
  private ConveyorSubsystem conveyor;
  private HoodSubsystem hood;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private SwerveSubsystem swerve;

  private Collection<TalonFX> talons = new ArrayList<TalonFX>();
  

  public PlayMusic(int selection, ConveyorSubsystem conveyor, HoodSubsystem hood, IntakeSubsystem intake, ShooterSubsystem shooter, SwerveSubsystem swerve){
    songSelection = selection;
    this.conveyor = conveyor;
    this.hood  = hood;
    this.intake = intake;
    this.shooter = shooter;
    this.swerve = swerve;

    addTalons();
    
    orchestra = new Orchestra(talons);
    
}

    public void addTalons(){
      // talons.add(conveyor.conveyorMotor);
      talons.add(intake.leftIntakeMotor);
      talons.add(intake.rightIntakeMotor);
      // talons.add(swerve.backRight.turningMotor);
      // talons.add(swerve.backRight.driveMotor);
      // talons.add(swerve.frontRight.turningMotor);
      // talons.add(swerve.frontRight.driveMotor);
      // talons.add(swerve.backLeft.driveMotor);
      // talons.add(swerve.backLeft.turningMotor);
      // talons.add(swerve.frontLeft.turningMotor);
      // talons.add(swerve.frontLeft.driveMotor);
    }
      

    private String[] songs = {
        Filesystem.getDeployDirectory().toString() + "/music/rushE.chrp",
        Filesystem.getDeployDirectory().toString() + "/music/rickRoll.chrp",
        Filesystem.getDeployDirectory().toString() + "/music/firel.chrp",
        Filesystem.getDeployDirectory().toString() + "/music/susl.chrp"
    };

    
    private int songSelection;

 

    public void loadMusic(){
      orchestra.loadMusic(songs[songSelection]);
  }

  
  @Override
  public void initialize() {
      loadMusic();
      orchestra.play();
  }

  
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
