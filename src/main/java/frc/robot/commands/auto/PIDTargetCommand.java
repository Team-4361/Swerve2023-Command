package frc.robot.commands.auto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.camera.PhotonCameraModule;

import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.util.math.ExtendedMath.inTolerance;


/**
 * This {@link PIDTargetCommand} is designed to <b>automatically</b> move the {@link Robot} to a specified
 * {@link AprilTag}, either the closest detectable, or a manually-specified ID tag (1-8 in FRC 2023 Game). It also
 * supports a customizable {@link Pose2d} offset, containing Left/Right (-X,+X), Forward/Reverse (-Y,+Y), and
 * a specified Angle (CW positive).
 *
 * @author Eric Gold
 */
public class PIDTargetCommand extends CommandBase {
    private final PIDController controller = new PIDController(0.1, 0, 0);
    private final ProfiledPIDController turnController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));

    public static final Pose2d DEFAULT_POSE = new Pose2d(
            new Translation2d(0, 2),
            new Rotation2d(0)
    );

    private final Pose2d poseOffset;
    private final Supplier<Pose2d> poseSupplier;

    private Pose2d trackedPose = new Pose2d();

    public PIDTargetCommand(Supplier<Pose2d> poseSupplier, Pose2d poseOffset) {
        this.poseSupplier = poseSupplier;
        this.poseOffset = poseOffset;

        addRequirements(Robot.swerveDrive);
    }

    public PIDTargetCommand(PhotonCameraModule camera) {
        this(camera::getTrackedPose, DEFAULT_POSE);
    }

    @Override
    public void execute() {
        trackedPose = poseSupplier.get();

        Robot.swerveDrive.robotDrive(
                !inTolerance(trackedPose.getX(), poseOffset.getX(), 2) ?
                        clamp(controller.calculate(trackedPose.getX(), poseOffset.getX()), -0.25, 0.25)
                        : 0,
                !inTolerance(trackedPose.getY(), poseOffset.getY(), 4) ?
                        clamp(controller.calculate(trackedPose.getY(), poseOffset.getY()), -0.25, 0.25)
                        : 0,
                !inTolerance(trackedPose.getRotation().getRadians(), Math.PI, 4) ?
                        clamp(turnController.calculate(trackedPose.getRotation().getRadians(), Math.PI), -0.05, 0.05)
                        : 0,
                0
        );
    }

    @Override
    public boolean isFinished() {
        return inTolerance(trackedPose.getX(), poseOffset.getX(), 2) && inTolerance(trackedPose.getY(), poseOffset.getY(), 4) && inTolerance(trackedPose.getRotation().getRadians(), Math.PI, 4);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }
}
