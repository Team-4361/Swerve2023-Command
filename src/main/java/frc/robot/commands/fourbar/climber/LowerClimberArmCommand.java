package frc.robot.commands.fourbar.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ClimberArmValues.CLIMBER_ROTATION_SPEED;

public class LowerClimberArmCommand extends CommandBase {

    public LowerClimberArmCommand(){
        addRequirements(Robot.climberSubsystem);
    }
    @Override
    public void initialize(){
        Robot.climberSubsystem.getRotation().translateMotor(-CLIMBER_ROTATION_SPEED);
    }
    @Override
    public void end(boolean interrupted){
        Robot.climberSubsystem.getRotation().translateMotor(0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
