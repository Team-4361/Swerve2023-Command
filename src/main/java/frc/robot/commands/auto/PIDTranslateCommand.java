package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.pid.VariablePose2d;

import java.util.function.Supplier;

import static frc.robot.Constants.AutoValues.*;
import static frc.robot.util.math.ExtendedMath.inTolerance;

public class PIDTranslateCommand extends CommandBase {
    private final VariablePose2d desiredPose;
    private VariablePose2d currentPose;

    public static final double DEFAULT_MAXIMUM_SPEED = 0.4;

    public PIDTranslateCommand(VariablePose2d pose) {
        this.desiredPose = pose;
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        currentPose = VariablePose2d.fromPose(Robot.swerveDrive.getPose());

        Robot.swerveDrive.autoDrive(
                desiredPose.hasX() ? MathUtil.clamp(X_CONTROLLER.calculate(currentPose.getX(), desiredPose.getX()), -DEFAULT_MAXIMUM_SPEED, DEFAULT_MAXIMUM_SPEED) : 0,
                desiredPose.hasY() ? MathUtil.clamp(Y_CONTROLLER.calculate(currentPose.getY(), desiredPose.getY()), -DEFAULT_MAXIMUM_SPEED, DEFAULT_MAXIMUM_SPEED) : 0,
                desiredPose.hasRotation() ? MathUtil.clamp(HEADING_CONTROLLER.calculate(currentPose.getDegrees(), desiredPose.getDegrees()), -DEFAULT_MAXIMUM_SPEED, DEFAULT_MAXIMUM_SPEED) : 0
        );
    }
    
    public boolean inTolerance(Supplier<Boolean> hasValue, Supplier<Double> value, Supplier<Double> target, double tolerance) {
        if (!hasValue.get()) return true;
        return ExtendedMath.inTolerance(target.get(), value.get(), tolerance);
    }

    @Override
    public boolean isFinished() {
        return inTolerance(desiredPose::hasX, currentPose::getX, desiredPose::getX, 1) &&
                inTolerance(desiredPose::hasY, currentPose::getY, desiredPose::getY, 1) &&
                inTolerance(desiredPose::hasRotation, currentPose::getDegrees, desiredPose::getDegrees, 2);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
