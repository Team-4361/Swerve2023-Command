package frc.robot.commands.fourbar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class PlaceObjectCommand extends CommandBase {
    public PlaceObjectCommand(){
        addRequirements(Robot.armSubsystem, Robot.gripperSubsystem, Robot.wristSubsystem);
    }

}
