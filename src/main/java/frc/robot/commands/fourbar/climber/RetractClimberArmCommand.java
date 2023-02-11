package frc.robot.commands.fourbar.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class RetractClimberArmCommand extends CommandBase {

    public RetractClimberArmCommand(){
        addRequirements(Robot.arm);
    }
    @Override
    public void initialize(){
        Robot.arm.getExtension().translateMotor(-0.5);
    }
    @Override
    public void end(boolean interrupted){
        Robot.arm.getExtension().translateMotor(0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
