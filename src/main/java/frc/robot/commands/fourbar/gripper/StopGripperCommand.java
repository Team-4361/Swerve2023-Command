package frc.robot.commands.fourbar.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class StopGripperCommand extends CommandBase {

    public StopGripperCommand(){
        addRequirements(Robot.gripperSubsystem);
    }
    @Override
    public void initialize(){
        Robot.gripperSubsystem.stopGripper();
    }
}
