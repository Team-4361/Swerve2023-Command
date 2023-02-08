package frc.robot.commands.fourbar;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.fourbar.arm.RaiseArmCommand;
import frc.robot.commands.fourbar.gripper.OpenGripperCommand;
import frc.robot.commands.fourbar.wrist.RaiseWristCommand;

public class OpenFourBarCommand extends ParallelRaceGroup {

    public OpenFourBarCommand(){
        super(
                new RaiseArmCommand(),
                new OpenGripperCommand(),
                new RaiseWristCommand()
        );
    }
}
