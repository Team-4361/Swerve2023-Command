package frc.robot.commands.assist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.auto.RotateMode;

import java.util.Objects;

import static frc.robot.Constants.AutoValues.HEADING_CONTROLLER;
import static frc.robot.util.auto.RotateMode.ABSOLUTE;
import static frc.robot.util.math.ExtendedMath.inTolerance;

public class PIDRotateCommand extends CommandBase {
    public static final double DEFAULT_ROTATE_SPEED = 0.5;

    private final RotateMode mode;
    private final double target;

    private double offset = 0, angle = 0, speed;

    public PIDRotateCommand(double target, double speed, RotateMode mode) {
        this.target = target;
        this.mode = mode;
        this.speed = speed;
        addRequirements(Robot.swerveDrive);
    }

    public PIDRotateCommand maxSpeed(double speed) {
        this.speed = speed;
        return this;
    }

    public PIDRotateCommand(double target) { this(target, DEFAULT_ROTATE_SPEED, ABSOLUTE); }

    @Override
    public void initialize() {
        offset = Robot.swerveDrive.getRobotHeading().getDegrees();
    }

    @Override
    public void execute() {
        if (Objects.requireNonNull(mode) == RotateMode.RELATIVE) {
            angle = offset + Robot.swerveDrive.getRobotHeading().getDegrees();
        } else {
            angle = Robot.swerveDrive.getRobotHeading().getDegrees();
        }

        Robot.swerveDrive.autoDrive(0, 0,
                MathUtil.clamp(HEADING_CONTROLLER.calculate(angle, target), -speed, speed)
        );
    }

    @Override
    public boolean isFinished() {
        return inTolerance(angle, target, 2);
    }
}
