package frc.robot.commands.fourbar;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.fourbar.arm.LowerArmCommand;
import frc.robot.commands.fourbar.gripper.CloseGripperCommand;
import frc.robot.commands.fourbar.wrist.LowerWristCommand;

public class CloseFourBarCommand extends ParallelRaceGroup {

    public CloseFourBarCommand(){
        super(
                new LowerArmCommand(),
                new CloseGripperCommand(),
                new LowerWristCommand()
        );
    }
}
