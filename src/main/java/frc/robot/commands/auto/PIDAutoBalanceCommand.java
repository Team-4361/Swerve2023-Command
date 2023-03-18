package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.AutoValues.PITCH_CONTROLLER;
import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class PIDAutoBalanceCommand extends CommandBase {
    public PIDAutoBalanceCommand() {
        addRequirements(Robot.swerveDrive);
        PITCH_CONTROLLER.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        Robot.swerveDrive.autoDrive(
                MathUtil.clamp(
                        PITCH_CONTROLLER.calculate(new Rotation2d(Robot.swerveDrive.gyro.getPitch()).getDegrees(), 0), -0.35, 0.35
                ),
                0,
                0
        );
    }

    @Override
    public boolean isFinished() {
        //return inTolerance(0, Robot.swerveDrive.gyro.getPitch(), 2);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
