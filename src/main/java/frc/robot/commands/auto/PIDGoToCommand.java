package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.AutoValues.*;

public class PIDGoToCommand extends CommandBase {
    private final Pose2d desiredPose;

    public static final double DEFAULT_MAXIMUM_SPEED = 0.5;

    public PIDGoToCommand(Pose2d pose) {
        this.desiredPose = pose;
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        Pose2d currentPose = Robot.swerveDrive.getPose();
        Robot.swerveDrive.autoDrive(
                MathUtil.clamp(X_CONTROLLER.calculate(currentPose.getX(), desiredPose.getX()), -DEFAULT_MAXIMUM_SPEED, DEFAULT_MAXIMUM_SPEED),
                MathUtil.clamp(Y_CONTROLLER.calculate(currentPose.getY(), desiredPose.getY()), -DEFAULT_MAXIMUM_SPEED, DEFAULT_MAXIMUM_SPEED),
                0
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
