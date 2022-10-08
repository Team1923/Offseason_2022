package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class OneMoreBar extends SequentialCommandGroup {

    //NEEDS TO BE BINDED TO DRIVER CONTROLLER LEFT BUMPER. IT'S CURRENTLY BEING USED FOR TEST COMMANDS. 
    public OneMoreBar(HoodSubsystem HOOD_SUBSYSTEM, ClimbSubsystem CLIMB_SUBSYSTEM, Supplier<Boolean> commit,
            ConveyorSubsystem conveyor, ShooterSubsystem shooter, SwerveSubsystem swerve, IntakeSubsystem intake) {
        addCommands(
                new ParallelRaceGroup(
                        new HoodSingleSetpointCommand(HOOD_SUBSYSTEM, 36000),
                        new ClimbApplyVoltage(CLIMB_SUBSYSTEM, .5)),
                new ParallelRaceGroup(
                        new HoodHoldPosition(HOOD_SUBSYSTEM, 9000),
                        new ArmsToPosition(CLIMB_SUBSYSTEM, ClimbConstants.maxArmPositionTicks) 
                ),
                new ParallelRaceGroup(
                        new ArmsToPosition(CLIMB_SUBSYSTEM, 0),
                        new SequentialCommandGroup(
                                new HoodSingleSetpointCommand(HOOD_SUBSYSTEM, 40100),
                                new HoodApplyVoltage(HOOD_SUBSYSTEM, 0.1)))

        );
    }
}
