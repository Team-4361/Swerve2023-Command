package frc.robot.commands.assist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.Chassis.CHASSIS_MAX_SPEED;
import static frc.robot.commands.auto.PIDTargetCommand.inTolerance;

public class PIDRotateCommand extends CommandBase {
    private final ProfiledPIDController turnController;
    private final double target;

    public PIDRotateCommand(double target) {
        turnController = new ProfiledPIDController(0.1, 0, 0,
                new TrapezoidProfile.Constraints(0.5, 0.5));
        this.target = target;
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        Robot.swerveDrive.drive(0, 0, turnController.calculate(
                Robot.swerveDrive.getRobotHeading().getDegrees(),
                target
        ) * CHASSIS_MAX_SPEED);
    }

    @Override
    public boolean isFinished() {
        return inTolerance(Robot.swerveDrive.getRobotHeading().getDegrees(), target, 2);
    }
}
