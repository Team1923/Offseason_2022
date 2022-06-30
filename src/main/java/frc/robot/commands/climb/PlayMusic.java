// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class PlayMusic extends CommandBase {
  private Orchestra orchestra;
    private WPI_TalonFX [] motors = {
        new WPI_TalonFX(DriveConstants.kFrontLeftDriveMotorPort),
        new WPI_TalonFX(DriveConstants.kFrontLeftTurningMotorPort),
        new WPI_TalonFX(DriveConstants.kFrontRightDriveMotorPort),
        new WPI_TalonFX(DriveConstants.kFrontRightTurningMotorPort),
        new WPI_TalonFX(DriveConstants.kBackLeftDriveMotorPort),
        new WPI_TalonFX(DriveConstants.kBackLeftTurningMotorPort),
        new WPI_TalonFX(DriveConstants.kBackRightDriveMotorPort),
        new WPI_TalonFX(DriveConstants.kBackRightTurningMotorPort),
        new WPI_TalonFX(IntakeConstants.leftIntakeMotorID),
        new WPI_TalonFX(IntakeConstants.rightIntakemotorID),
        new WPI_TalonFX(ConveyorConstants.conveyorMotorID),
        new WPI_TalonFX(ShooterConstants.leftShooterMotorID),
        new WPI_TalonFX(ShooterConstants.rightShooterMotorID),
        new WPI_TalonFX(HoodConstants.hoodMotorID)
    };    

    private String[] songs = {
        Filesystem.getDeployDirectory().toString() + "/music/rushE.chrp"
    };

    private int songSelection;

    public PlayMusic(int selection){
        songSelection = selection;
        Collection<TalonFX> talons = new ArrayList<TalonFX>();
        for(int i = 0; i < motors.length; i++){
            talons.add(motors[i]);
        }
        orchestra = new Orchestra(talons);
        loadMusic();
        orchestra.play();
    }

    public void loadMusic(){
      orchestra.loadMusic(songs[songSelection]);
  }

  
  @Override
  public void initialize() {}

  
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
