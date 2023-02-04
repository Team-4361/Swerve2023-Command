package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    public static final int CAMERA_BUFFER_MILLIS = 500;

    private PhotonTrackedTarget trackedTarget;
    private Transform3d targetTransform;
    private Pose2d trackedPose;
    public boolean targetFound = false;

    private long lastFoundMillis;

    public CameraSubsystem() {
        this.camera = new PhotonCamera(Constants.FrontCamera.CAMERA_NAME);
        this.trackedPose = new Pose2d();
        this.targetTransform = new Transform3d();
        this.trackedTarget = new PhotonTrackedTarget();
        this.lastFoundMillis = System.currentTimeMillis();
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

        if (result.hasTargets()) {
            targetFound = true;
            trackedTarget = result.getBestTarget();
            targetTransform = trackedTarget.getBestCameraToTarget();
            
            trackedPose = new Pose2d(
                    new Translation2d(Math.abs(targetTransform.getY()), Math.abs(-targetTransform.getX())),
                    new Rotation2d((Units.degreesToRadians(180)-Units.degreesToRadians(trackedTarget.getYaw()))%Units.degreesToRadians(360))
            );
             

            /* 
            trackedPose = new Pose2d(
                    new Translation2d(targetTransform.getX(), targetTransform.getY()),
                    new Rotation2d(Math.PI-Units.degreesToRadians(trackedTarget.getYaw())%(2*Math.PI))
            );
            */

            lastFoundMillis = System.currentTimeMillis();
        } else if (System.currentTimeMillis() >= lastFoundMillis+CAMERA_BUFFER_MILLIS){
            targetTransform = new Transform3d();
            trackedPose = new Pose2d();
            targetFound = false;
        }

        SmartDashboard.putString("Camera Pose", trackedPose.toString());
        SmartDashboard.putString("Camera Transform", targetTransform.toString());
        SmartDashboard.putBoolean("Camera Target Found", targetFound);
    }
}
