package frc.robot.commands.fourbar.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.FourBarArmValues.ARM_PRESETS;
import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class RaiseArmCommand extends CommandBase {
    public RaiseArmCommand() {
        addRequirements(Robot.armSubsystem);
    }

    @Override
    public void initialize() {
        Robot.armSubsystem.setTarget(ARM_PRESETS
                .nextPreset() // go to the next preset
                .getCurrentPreset() // get that preset number
        );
    }

    @Override
    public boolean isFinished() {
        return Robot.armSubsystem.atTarget();
    }
}
