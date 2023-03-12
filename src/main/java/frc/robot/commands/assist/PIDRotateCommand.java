package frc.robot.commands.assist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class PIDRotateCommand extends CommandBase {
    private final PIDController turnController;
    private final double target;

    public PIDRotateCommand(double target) {
        turnController = new PIDController(0.1, 0, 0);
        this.target = target;
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        Robot.swerveDrive.autoDrive(0, 0, 
                MathUtil.clamp(turnController.calculate(
                    Robot.swerveDrive.getRobotHeading().getDegrees(),
                    target), -0.5, 0.5)
        );
    }

    @Override
    public boolean isFinished() {
        return inTolerance(Robot.swerveDrive.getRobotHeading().getDegrees(), target, 2);
    }
}
