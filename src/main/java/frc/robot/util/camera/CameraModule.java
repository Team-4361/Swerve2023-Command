package frc.robot.util.camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraModule extends PhotonCamera {

    public static final int CAMERA_BUFFER_MILLIS = 500;

    private PhotonTrackedTarget trackedTarget;
    private Transform3d targetTransform;
    private Pose2d trackedPose;
    private int[] aprilTagIncludes = new int[]{};
    private boolean targetFound = false;

    private long lastFoundMillis = System.currentTimeMillis();

    public CameraModule onlyIncludeAprilTags(int... aprilTags) {
        this.aprilTagIncludes = aprilTags;
        return this;
    }

    public CameraModule resetAprilTags() {
        this.aprilTagIncludes = new int[]{};
        return this;
    }

    public Pose2d getTrackedPose() {
        return trackedPose;
    }

    public boolean isTargetFound() {
        return targetFound;
    }

    public void update() {
        PhotonPipelineResult result = this.getLatestResult();

        if (result.hasTargets()) {
            targetFound = true;
            trackedTarget = result.getBestTarget();
            targetTransform = trackedTarget.getBestCameraToTarget();

            trackedPose = new Pose2d(
                    new Translation2d(targetTransform.getX(), targetTransform.getY()),
                    new Rotation2d(Math.PI- Units.degreesToRadians(trackedTarget.getYaw())%(2*Math.PI))
            );

            lastFoundMillis = System.currentTimeMillis();
        } else if (System.currentTimeMillis() >= lastFoundMillis+CAMERA_BUFFER_MILLIS){
            targetTransform = new Transform3d();
            trackedPose = new Pose2d();
            targetFound = false;
        }

        SmartDashboard.putString("Photon: " + getName() + " Pose", trackedPose.toString());
        SmartDashboard.putString("Photon: " + getName() + " Transform", targetTransform.toString());
        SmartDashboard.putBoolean("Photon: " + getName() + " Target Found", targetFound);
    }

    public CameraModule(NetworkTableInstance instance, String cameraName) {
        super(instance, cameraName);
    }

    public CameraModule(String cameraName) {
        super(cameraName);
    }

}
