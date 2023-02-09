package frc.robot.util.camera;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraModule extends PhotonCamera {
    private PhotonTrackedTarget target;
    private Transform2d targetTransform;
    private Pose2d targetPose;

    private final CameraConfig config;

    private boolean targetFound;
    private long lastFoundMillis;

    public static final int CAMERA_BUFFER_MILLIS = 500;
    public PhotonTrackedTarget getTarget() { return target; }
    public Transform2d getTargetTransform() { return targetTransform; }
    public Pose2d getTargetPose() { return targetPose; }
    public boolean isTargetFound() { return targetFound; }

    /**
     * Constructs a PhotonCamera from a root table.
     * @param instance   The NetworkTableInstance to pull data from. This can be a custom instance in
     *                   simulation, but should *usually* be the default NTInstance from
     *                   NetworkTableInstance::getDefault
     */
    public CameraModule(NetworkTableInstance instance, CameraConfig config) {
        super(instance, config.getCameraName());

        this.config = config;
        this.target = new PhotonTrackedTarget();
        this.targetTransform = new Transform2d();
        this.targetPose = new Pose2d();
        this.targetFound = false;
        this.lastFoundMillis = System.currentTimeMillis();
    }

    /**
     * Constructs a PhotonCamera from the name of the camera.
     */
    public CameraModule(CameraConfig config) {
        this(NetworkTableInstance.getDefault(), config);
    }

    public void update() {
        /*
        PhotonPipelineResult result = getLatestResult();

        if (result.hasTargets()) {
            targetFound = true;
            target = result.getBestTarget();
            targetTransform = new Transform2d(
                    target.getBestCameraToTarget().getTranslation().toTranslation2d(),
                    target.getBestCameraToTarget().getRotation().toRotation2d()
            );

            if (targetTransform.equals(new Transform3d())) {
                // The target exists but there is no transform, most likely in 2D mode
                targetTransform = new Transform2d(
                        new Translation2d(
                                PhotonUtils.calculateDistanceToTargetMeters(config.getCameraHeightMeters(), config.getTargetHeightMeters(), config.getCameraPitch(), target.getPitch()),


                ));
            }
        }
        *
         */
    }
}
