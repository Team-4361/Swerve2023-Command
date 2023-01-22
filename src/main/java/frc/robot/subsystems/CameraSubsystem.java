package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FrontCamera;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.nio.file.Path;

import static frc.robot.subsystems.SwerveDriveSubsystem.inTolerance;

public class CameraSubsystem extends SubsystemBase {
    private final PhotonCamera camera;

    public static final int CAMERA_BUFFER_MILLIS = 500;
    public static final Pose2d DEFAULT_POSE_OFFSET = new Pose2d(
            new Translation2d(-2, 0),
            new Rotation2d(0)
    );

    private PhotonTrackedTarget trackedTarget;
    private Transform3d targetTransform;
    private Pose3d trackedPose;
    public boolean targetFound = false;

    private final SimVisionSystem simCamera = new SimVisionSystem(
            FrontCamera.CAMERA_NAME,
            68.5,
            new Transform3d(
                    new Translation3d(0, 0, FrontCamera.CAMERA_HEIGHT_METERS),
                    new Rotation3d(0, FrontCamera.CAMERA_PITCH, 0)
            ),
            20,
            1280,
            720,
            10
    );


    private long lastFoundMillis;

    public CameraSubsystem() throws IOException {
        this.camera = new PhotonCamera(FrontCamera.CAMERA_NAME);
        this.trackedPose = new Pose3d();
        this.targetTransform = new Transform3d();
        this.trackedTarget = new PhotonTrackedTarget();
        this.lastFoundMillis = System.currentTimeMillis();

        simCamera.addVisionTargets(AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile));
    }

    public Pose2d getTrackedPose() {
        return trackedPose.toPose2d();
    }

    public boolean isTargetFound() {
        return targetFound;
    }

    private void processTarget(PhotonTrackedTarget target) {
        this.trackedTarget = target;
        this.targetTransform = trackedTarget.getBestCameraToTarget();
        this.trackedPose = new Pose3d(
                targetTransform.getTranslation(),
                targetTransform.getRotation()
        );
        this.targetFound = true;
    }


    @Override
    public void periodic() {
        simCamera.processFrame(Robot.swerveDrive.getRobotPose());

        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {

            processTarget(result.getBestTarget());
            this.lastFoundMillis = System.currentTimeMillis();


        } else if (System.currentTimeMillis() >= lastFoundMillis+CAMERA_BUFFER_MILLIS) {
            // Use a half-second buffer on the event the Pipeline cannot keep up with the PID loop.
            this.targetTransform = new Transform3d();
            this.trackedPose = new Pose3d();
            this.targetFound = false;
        }

        SmartDashboard.putString("Camera Pose", trackedPose.toString());
        SmartDashboard.putString("Camera Transform", targetTransform.toString());
        SmartDashboard.putBoolean("Camera Target Found", targetFound);
    }
}
