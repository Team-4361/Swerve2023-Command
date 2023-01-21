package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDriveSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.concurrent.atomic.AtomicReference;

import static edu.wpi.first.math.MathUtil.clamp;

/**
 * This {@link PIDTargetCommand} is designed to <b>automatically</b> move the {@link Robot} to a specified
 * {@link AprilTag}, either the closest detectable, or a manually-specified ID tag (1-8 in FRC 2023 Game). It also
 * supports a customizable {@link Pose2d} offset, containing Left/Right (-X,+X), Forward/Reverse (-Y,+Y), and
 * a specified Angle (CW positive).
 *
 * @author Eric Gold
 */
public class PIDTargetCommand extends CommandBase {
    public static final Pose2d DEFAULT_POSE = new Pose2d(
            new Translation2d(10, 0),
            new Rotation2d(0)
    );
    private final Pose2d tagOffset;
    private long foundTime = System.currentTimeMillis();

    public PIDTargetCommand(Pose2d tagOffset) {
        this.tagOffset = tagOffset;
    }

    public PIDTargetCommand() {
        this.tagOffset = DEFAULT_POSE;
    }
    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() { addRequirements(Robot.swerveDrive); }

    @Override
    public void execute() {
        if (Robot.cameraSubsystem.isTargetFound()) {
            Robot.swerveDrive.driveToPose(Robot.cameraSubsystem.getTrackedPose(), tagOffset);
        } else {
            Robot.swerveDrive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.isCorrectPose(Robot.cameraSubsystem.getTrackedPose(), tagOffset);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
