package frc.robot.commands.fourbar.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.FourBarWristValues.WRIST_PRESETS;
import static frc.robot.Constants.FourBarWristValues.WRIST_ANGLE_TOLERANCE;
import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class RaiseWristCommand extends CommandBase {

    public RaiseWristCommand() {
        addRequirements(Robot.wristSubsystem);
    }

    @Override
    public void initialize() {
        Robot.wristSubsystem.setAngleDeg(
                WRIST_PRESETS.nextPreset().getCurrentPreset()
        );
    }

    @Override
    public boolean isFinished() {
        return inTolerance(WRIST_PRESETS.getCurrentPreset(), Robot.wristSubsystem.getAngleDeg(), WRIST_ANGLE_TOLERANCE);
    }
}
