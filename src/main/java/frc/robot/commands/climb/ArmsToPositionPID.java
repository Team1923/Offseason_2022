// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ArmsToPositionPID extends CommandBase {

  private double tickThreshold = 1024;

  private int loopsInsideAllowableError;
  private int loopsThreshold = 5;

  private double kFF = .25; 

  PIDController controller;
  ClimbSubsystem climb;
  double goal_ticks;
  double start_pos;
  /** Creates a new ArmsToPositionPID. */
  public ArmsToPositionPID(ClimbSubsystem c, double goal) {
    this.climb = c;
    this.goal_ticks = -goal;

    controller = new PIDController(.0001, 0, .000001);

    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(goal_ticks);
    System.out.println(start_pos);
    this.start_pos = climb.getLeftClimbEncoderPosition();
    if(goal_ticks < start_pos) {
      kFF = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // SmartDashboar.putNumber("FF", kFF);
    double output = controller.calculate(climb.getLeftClimbEncoderPosition(), goal_ticks) + kFF;
    if(output > 1) {
      output = 1;
    }
   // SmartDashboar.putNumber("Climber Output", output);
    climb.set(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stop();
    System.out.println("EXITED PID!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(climb.getLeftClimbEncoderPosition()-goal_ticks) < tickThreshold) {
      loopsInsideAllowableError++;
    } else {
      loopsInsideAllowableError = 0;
    }

    // If we have been within the threshold for a certain number of loops, we deem that the setpoint has been reached, and the command exits.
    return loopsInsideAllowableError >= loopsThreshold;
  }
}
