package frc.robot.commands.fourbar.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.FourBarArmValues.ARM_LOWER_ROTATION;
import static frc.robot.Constants.FourBarArmValues.ARM_ROTATION_TOLERANCE;

public class LowerArmCommand extends CommandBase {

    public LowerArmCommand() {
        addRequirements(Robot.armSubsystem);
    }

    @Override
    public void initialize() {
        Robot.armSubsystem.setTarget(ARM_LOWER_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return Robot.armSubsystem.atTarget();
    }
}
