package frc.robot.commands.fourbar.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.FourBarArmValues.ARM_ANGLE_DEGREES;
import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class RaiseArmCommand extends CommandBase {
    public RaiseArmCommand(){
        addRequirements(Robot.armSubsystem);
    }
    @Override
    public void initialize(){
        Robot.armSubsystem.setTargetDeg(ARM_ANGLE_DEGREES);
    }
    @Override
    public boolean isFinished(){
        return inTolerance(ARM_ANGLE_DEGREES, Robot.armSubsystem.getArmAngle().getDegrees(), 2);
    }
}
