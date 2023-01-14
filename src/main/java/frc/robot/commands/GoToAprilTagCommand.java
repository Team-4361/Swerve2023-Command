package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.Constants.Camera.*;
import static frc.robot.Robot.camera;

public class GoToAprilTagCommand extends CommandBase {
    private final PIDController aprilTagController;
    private final double targetDistance, targetYaw;
    private double currentDistance, currentYaw;

    public GoToAprilTagCommand(double targetDistance, double targetYaw) {
        this.aprilTagController = new PIDController(0, 0, 0);

        this.targetDistance = targetDistance;
        this.targetYaw = targetYaw;

        this.currentDistance = 0;
        this.currentYaw = 0;
    }

    @Override
    public void initialize() {
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        PhotonPipelineResult latestResult = camera.getLatestResult();

        if (latestResult.hasTargets()) {
            // Calculate the current distance and yaw and process it.
            PhotonTrackedTarget trackedTarget = latestResult.getBestTarget();

            currentYaw = trackedTarget.getYaw();
            currentDistance = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH,
                    trackedTarget.getPitch()
            );


            Robot.swerveDrive.robotDrive(
                    0,
                    clamp(aprilTagController.calculate(currentDistance, targetDistance), -0.4, 0.4),
                    clamp(aprilTagController.calculate(currentYaw, targetYaw), -0.4, 0.4),
                    0
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
