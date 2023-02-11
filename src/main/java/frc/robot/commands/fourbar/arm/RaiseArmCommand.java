package frc.robot.commands.fourbar.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.FourBarArmValues.ARM_RAISE_ROTATION;
import static frc.robot.Constants.FourBarArmValues.ARM_ROTATION_TOLERANCE;
import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class RaiseArmCommand extends CommandBase {
    public RaiseArmCommand() {
        addRequirements(Robot.armSubsystem);
    }

    @Override
    public void initialize() {
        Robot.armSubsystem.setTarget(ARM_RAISE_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return Robot.armSubsystem.atTarget();
    }
}
