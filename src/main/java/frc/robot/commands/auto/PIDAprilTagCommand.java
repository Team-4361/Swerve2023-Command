package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.concurrent.atomic.AtomicReference;

import static edu.wpi.first.math.MathUtil.clamp;

/**
 * This {@link PIDAprilTagCommand} is designed to <b>automatically</b> move the {@link Robot} to a specified
 * {@link AprilTag}, either the closest detectable, or a manually-specified ID tag (1-8 in FRC 2023 Game). It also
 * supports a customizable {@link Pose2d} offset, containing Left/Right (-X,+X), Forward/Reverse (-Y,+Y), and
 * a specified Angle (CW positive).
 *
 * @author Eric Gold
 */
public class PIDAprilTagCommand extends CommandBase {
    public static final PIDController DEFAULT_PID_CONTROLLER = new PIDController(0.1, 0, 0);
    public static final Pose2d DEFAULT_POSE = new Pose2d(
            new Translation2d(0, -2),
            new Rotation2d(0)
    );

    public static final double DEFAULT_MAX_TURN_POWER = 0.5;
    public static final double DEFAULT_MAX_DRIVE_POWER = 0.5;

    private final PIDController tagController;
    private final Pose2d tagOffset;
    private final int desiredID;

    private Transform3d targetTransform;
    private AtomicReference<PhotonTrackedTarget> trackedTarget;
    private double maxTurnPower = DEFAULT_MAX_TURN_POWER;
    private double maxDrivePower = DEFAULT_MAX_DRIVE_POWER;

    /**
     * Checks if the actual value is within a specified tolerance of the expected value
     * @param expected The value to be expected.
     * @param actual The actual value.
     * @param tolerance The maximum error or tolerance that the value can be offset to still be true.
     * @return True/false depending on tolerance.
     */
    private static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    /**
     * Constructs a new {@link PIDAprilTagCommand}, using a specified {@link PIDController}, {@link Pose2d}, and
     * desired {@link AprilTag} ID.
     *
     * @param tagController The {@link PIDController} to use for all calculations.
     * @param tagOffset The {@link Pose2d} to use for the AprilTag offset,
     *                  (X+ right, Y+ dist towards (forward), CW+ angle)
     * @param desiredID The desired IDs to be detected to transport to
     *                   (-1 = best target)
     */
    public PIDAprilTagCommand(PIDController tagController, Pose2d tagOffset, int desiredID) {
        this.tagController = tagController;
        this.tagOffset = tagOffset;
        this.desiredID = desiredID;
    }

    /**
     * Constructs a new {@link PIDAprilTagCommand}, using a specified {@link PIDController}, {@link Pose2d}, and
     * navigate to the closest detected {@link AprilTag}.
     *
     * @param tagController The {@link PIDController} to use for all calculations.
     * @param tagOffset The {@link Pose2d} to use for the AprilTag offset,
     *                  (X+ right, Y+ dist away (forward), CW+ angle)
     */
    public PIDAprilTagCommand(PIDController tagController, Pose2d tagOffset) {
        this(tagController, tagOffset, -1);
    }

    /**
     * Constructs a new {@link PIDAprilTagCommand}, using a specified {@link PIDController}, and default {@link Pose2d}
     * with 6 foot clearance between Camera and Target and 0-degree angle.
     *
     * @param tagController The {@link PIDController} to use for all calculations.
     */
    public PIDAprilTagCommand(PIDController tagController) {
        this(tagController, DEFAULT_POSE, -1);
    }

    /**
     * Constructs a new {@link PIDAprilTagCommand}, using a specified {@link PIDController}, and desired IDs.
     *
     * @param tagOffset The {@link Pose2d} to use for the {@link AprilTag} offset,
     *                  (X+ right, Y+ dist towards (forward), CW+ angle)
     * @param desiredID The desired ID to be detected to transport to (-1 = best target)
     */
    public PIDAprilTagCommand(Pose2d tagOffset, int desiredID) {
        this(DEFAULT_PID_CONTROLLER, DEFAULT_POSE, desiredID);
    }

    /**
     * Constructs a new {@link PIDAprilTagCommand}
     */
    public PIDAprilTagCommand() {
        this(DEFAULT_PID_CONTROLLER, DEFAULT_POSE, -1);
    }

    /**
     * Constructs a new {@link PIDAprilTagCommand}, using a specified {@link Pose2d}.
     *
     * @param tagOffset The {@link Pose2d} to use for the {@link AprilTag} offset,
     *                  (X+ right, Y+ dist towards (forward), CW+ angle)
     */
    public PIDAprilTagCommand(Pose2d tagOffset) {
        this(DEFAULT_PID_CONTROLLER, tagOffset, -1);
    }

    public PIDAprilTagCommand setMaxDrivePower(double drivePower) {
        this.maxDrivePower = drivePower;
        return this;
    }

    public PIDAprilTagCommand setMaxTurnPower(double turnPower) {
        this.maxTurnPower = turnPower;
        return this;
    }

    public double getMaxDrivePower() { return this.maxDrivePower; }
    public double getMaxTurnPower() { return this.maxTurnPower; }

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
        Robot.swerveDrive.autoDrive(
                // TODO: some of these values may be flipped or inaccurate, tuning may be required.
                clamp(tagController.calculate(targetTransform.getY(), tagOffset.getX()), -maxDrivePower, maxDrivePower),
                clamp(tagController.calculate(targetTransform.getX(), tagOffset.getY()), -maxDrivePower, maxDrivePower),
                clamp(tagController.calculate(trackedTarget.get().getYaw(), tagOffset.getRotation().getDegrees()), -maxTurnPower, maxTurnPower)
        );
    }

    @Override
    public boolean isFinished() {
        return (
                inTolerance(targetTransform.getX(), tagOffset.getX(), 0.25) &&
                inTolerance(targetTransform.getY(), tagOffset.getY(), 0.25) &&
                inTolerance(trackedTarget.get().getYaw(), tagOffset.getRotation().getDegrees(), 5)
        );
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
