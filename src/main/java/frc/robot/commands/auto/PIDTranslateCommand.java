package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.AutoValues.*;

public class PIDTranslateCommand extends CommandBase {
    private final Translation2d desiredTranslation;
    private Translation2d currentTranslation;

    public static final double DEFAULT_MAXIMUM_SPEED = 0.4;

    public PIDTranslateCommand(Translation2d translation) {
        this.desiredTranslation = translation;
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        currentTranslation = new Translation2d(
                Robot.swerveDrive.getPose().getX(),
                Robot.swerveDrive.getPose().getY()
        );
        Robot.swerveDrive.autoDrive(
                MathUtil.clamp(X_CONTROLLER.calculate(currentTranslation.getX(), desiredTranslation.getX()), -DEFAULT_MAXIMUM_SPEED, DEFAULT_MAXIMUM_SPEED),
                MathUtil.clamp(Y_CONTROLLER.calculate(currentTranslation.getY(), desiredTranslation.getY()), -DEFAULT_MAXIMUM_SPEED, DEFAULT_MAXIMUM_SPEED),
                0
        );
    }
    
    private boolean xInTolerance() {
        return PIDTargetCommand.inTolerance(desiredTranslation.getX(), currentTranslation.getX(), 1);
    }

    private boolean yInTolerance() {
        return PIDTargetCommand.inTolerance(desiredTranslation.getY(), currentTranslation.getY(), 1);
    }

    @Override
    public boolean isFinished() {
        return xInTolerance() && yInTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
