package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonTrackedTarget trackedTarget;
    private Transform3d targetTransform;
    private Pose2d trackedPose;
    public boolean targetFound = false;


    public CameraSubsystem() {
        this.camera = new PhotonCamera(Constants.FrontCamera.CAMERA_NAME);
        this.trackedPose = new Pose2d();
        this.targetTransform = new Transform3d();
        this.trackedTarget = new PhotonTrackedTarget();
    }

    public Pose2d getTrackedPose() {
        return trackedPose;
    }

    public boolean isTargetFound() {
        return targetFound;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        targetFound = result.hasTargets();

        if (result.hasTargets()) {
            trackedTarget = result.getBestTarget();
            targetTransform = trackedTarget.getBestCameraToTarget();
            trackedPose = new Pose2d(
                    new Translation2d(targetTransform.getY(), targetTransform.getX()),
                    targetTransform.getRotation().toRotation2d()
            );
        } else {
            targetTransform = new Transform3d();
            trackedPose = new Pose2d();
        }

        SmartDashboard.putString("Camera Pose", trackedPose.toString());
        SmartDashboard.putString("Camera Transform", targetTransform.toString());
    }


}
