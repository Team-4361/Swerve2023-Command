package frc.robot.util.camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraModule extends PhotonCamera {

    public static final int CAMERA_BUFFER_MILLIS =  500;

    private final PhotonCameraConfig config;

    private Transform3d targetTransform;
    private Pose2d trackedPose;
    private boolean targetFound = false;


    private long lastFoundMillis = System.currentTimeMillis();

    public Pose2d getTrackedPose() { return trackedPose; }
    public PhotonCameraModule setPipeline(int index) {
        setPipelineIndex(index);
        return this;
    }

    public boolean isTargetFound() { return targetFound;  }

    public void update() {
        PhotonPipelineResult result = this.getLatestResult();

        if (result.hasTargets()) {
            targetFound = true;
            PhotonTrackedTarget trackedTarget = result.getBestTarget();
            targetTransform = trackedTarget.getBestCameraToTarget();

            // If the transform is equal to zero with a pitch/yaw existing, then its in 2D mode. Calculate the
            // distance using the trig formula.
            if (targetTransform.getX() == 0 && targetTransform.getY() == 0) {
                trackedPose = new Pose2d(
                        new Translation2d(
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        config.getCameraHeight(),
                                        config.getTargetHeight(),
                                        config.getCameraPitch(),
                                        trackedTarget.getPitch()),
                                0),
                        new Rotation2d(trackedTarget.getYaw())
                );
            } else {
                // The target is running in 3D/AprilTag Mode. Automatically determine it through the transform.
                trackedPose = new Pose2d(
                        new Translation2d(targetTransform.getX(), targetTransform.getY()),
                        new Rotation2d(Math.PI- Units.degreesToRadians(trackedTarget.getYaw())%(2*Math.PI))
                );
            }

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

    public PhotonCameraModule(NetworkTableInstance instance, PhotonCameraConfig config) {
        super(instance, config.getCameraName());
        this.config = config;
    }

    public PhotonCameraModule(PhotonCameraConfig config) {
        super(config.getCameraName());
        this.config = config;
    }

}
