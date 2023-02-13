package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RaiseClimberArmCommand extends CommandBase {

    public RaiseClimberArmCommand(){
        addRequirements(Robot.arm);
    }
    @Override
    public void initialize(){
        Robot.arm.getRotation().translateMotor(0.5);
    }
    @Override
    public void end(boolean interrupted){
        Robot.arm.getRotation().translateMotor(0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
