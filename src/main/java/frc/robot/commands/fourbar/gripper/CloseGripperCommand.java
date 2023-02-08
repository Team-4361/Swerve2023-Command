package frc.robot.commands.fourbar.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.FourBarGripperValues.GRIPPER_SPEED;

public class CloseGripperCommand extends CommandBase {

    public CloseGripperCommand(){
        addRequirements(Robot.gripperSubsystem);
    }
    @Override
    public void initialize(){
        Robot.gripperSubsystem.closeGripper(GRIPPER_SPEED);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean interrupted){
        Robot.gripperSubsystem.closeGripper(0);
    }
}
