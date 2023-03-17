package frc.robot.commands.assist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class PIDRotateCommand extends CommandBase {
    private final ProfiledPIDController controller = new ProfiledPIDController(0.01, 0, 0, new Constraints(0.5, 0.5));
    private final double target;

    private double offset = 0, angle = 0;

    public PIDRotateCommand(double target) {
        this.target = target;
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        offset = Robot.swerveDrive.getRobotHeading().getDegrees();
    }

    @Override
    public void execute() {
        angle = offset + Robot.swerveDrive.getRobotHeading().getDegrees();
        Robot.swerveDrive.autoDrive(0, 0, 
                MathUtil.clamp(controller.calculate(angle, target), -0.5, 0.5)
        );
    }

    @Override
    public boolean isFinished() {
        return inTolerance(angle, target, 2);
    }
}
