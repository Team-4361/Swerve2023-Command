package frc.robot.commands.fourbar.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ClimberArmValues.CLIMBER_EXTENSION_SPEED;

public class RetractClimberArmCommand extends CommandBase {

    public RetractClimberArmCommand(){
        addRequirements(Robot.climberSubsystem);
    }
    @Override
    public void initialize(){
        Robot.climberSubsystem.getExtension().translateMotor(-CLIMBER_EXTENSION_SPEED);
    }
    @Override
    public void end(boolean interrupted){
        Robot.climberSubsystem.getExtension().translateMotor(0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
