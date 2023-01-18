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
            new Translation2d(0, -2),
            new Rotation2d(0)
    );

    private final Pose2d tagOffset;
    private final int desiredID;

    private Transform3d targetTransform;
    private AtomicReference<PhotonTrackedTarget> trackedTarget;
    private Pose2d trackedPose;

    /**
     * Constructs a new {@link PIDTargetCommand}, using a specified {@link PIDController}, {@link Pose2d}, and
     * desired {@link AprilTag} ID.
     *
     * @param tagOffset The {@link Pose2d} to use for the AprilTag offset,
     *                  (X+ right, Y+ dist towards (forward), CW+ angle)
     * @param desiredID The desired IDs to be detected to transport to
     *                   (-1 = best target)
     */
    public PIDTargetCommand(Pose2d tagOffset, int desiredID) {
        this.tagOffset = tagOffset;
        this.desiredID = desiredID;
    }

    /**
     * Constructs a new {@link PIDTargetCommand}, using a specified {@link PIDController}, {@link Pose2d}, and
     * navigate to the closest detected {@link AprilTag}.
     *
     * @param tagOffset The {@link Pose2d} to use for the AprilTag offset,
     *                  (X+ right, Y+ dist away (forward), CW+ angle)
     */
    public PIDTargetCommand(Pose2d tagOffset) {
        this(tagOffset, -1);
    }

    /**
     * Constructs a new {@link PIDTargetCommand}, using a specified {@link PIDController}, and desired IDs.
     *
     * @param desiredID The desired ID to be detected to transport to (-1 = best target)
     */
    public PIDTargetCommand(int desiredID) {
        this(DEFAULT_POSE, desiredID);
    }

    /**
     * Constructs a new {@link PIDTargetCommand}
     */
    public PIDTargetCommand() { this(DEFAULT_POSE, -1); }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() { addRequirements(Robot.swerveDrive); }

    @Override
    public void execute() {
        // Get the latest result of the camera, which will include all detected targets.
        PhotonPipelineResult result = Robot.camera.getLatestResult();
         trackedTarget = new AtomicReference<>();

        // If no targets are detected, stop driving the Robot and don't proceed with calculations.
        if (!result.hasTargets()) {
            Robot.swerveDrive.stop();
            return;
        }

        if (desiredID == -1) {
            // No specific IDs were requested, so find the best target if any exist.
            trackedTarget.set(result.getBestTarget());
        } else {
            // A specific ID was requested, loop through all detected targets and see if any IDs are there.
            for (PhotonTrackedTarget target : result.targets) {
                if (target.getFiducialId() == desiredID) {
                    trackedTarget.set(target);
                    break;
                }
            }
        }

        // If no targets have been chosen due to the conditions, stop driving the Robot and don't proceed
        // with finding Translations.
        if (trackedTarget.get() == null) {
            Robot.swerveDrive.stop();
            return;
        }

        // Find the Transform3D of the Camera and convert the angles and distances into something that could
        // be processed. Using the "getBestCameraToTarget" value is okay as we filter out any IDs that don't
        // match the required setting.
        targetTransform = trackedTarget.get().getBestCameraToTarget();

        // Target Transform:
        // (+X = dist away, -X = dist towards)
        // (+Y = dist left, -Y = dist right)
        // (Z = angle)

        // Robot Transform:
        // (+X = dist right, -X = dist left)
        // (+Y = dist towards, -Y = dist away)
        // (Z = angle)
        trackedPose = new Pose2d(
                new Translation2d(targetTransform.getY(), targetTransform.getX()),
                targetTransform.getRotation().toRotation2d()
        );

        Robot.swerveDrive.driveToPose(trackedPose, tagOffset);
    }

    @Override
    public boolean isFinished() {
        return SwerveDriveSubsystem.isCorrectPose(trackedPose, tagOffset);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
