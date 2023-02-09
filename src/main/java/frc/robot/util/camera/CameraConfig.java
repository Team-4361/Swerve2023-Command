package frc.robot.util.camera;

import org.photonvision.PhotonUtils;

public class CameraConfig {
    private double cameraHeightMeters, targetHeightMeters, cameraPitch;
    private String cameraName;

    public double getCameraHeightMeters() {
        return cameraHeightMeters;
    }

    public CameraConfig setCameraHeightMeters(double cameraHeightMeters) {
        this.cameraHeightMeters = cameraHeightMeters;
        return this;
    }

    public double getTargetHeightMeters() {
        return targetHeightMeters;
    }

    public CameraConfig setTargetHeightMeters(double targetHeightMeters) {
        this.targetHeightMeters = targetHeightMeters;
        return this;
    }

    public double getCameraPitch() {
        return cameraPitch;
    }

    public CameraConfig setCameraPitch(double cameraPitch) {
        this.cameraPitch = cameraPitch;
        return this;
    }

    public String getCameraName() {
        return cameraName;
    }

    public CameraConfig setCameraName(String cameraName) {
        this.cameraName = cameraName;
        return this;
    }
}
