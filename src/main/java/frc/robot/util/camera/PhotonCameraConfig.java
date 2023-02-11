package frc.robot.util.camera;

public class PhotonCameraConfig {

    private double cameraHeight = 0;
    private double targetHeight = 0;
    private double cameraPitch = 0;
    private String cameraName = "photoncamera";
    private int aprilTagPipeline = 1, conePipeline = 2, cubePipeline = 3;

    public PhotonCameraConfig setCameraHeight(double cameraHeight) {
        this.cameraHeight = cameraHeight;
        return this;
    }

    public PhotonCameraConfig setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
        return this;
    }


    public PhotonCameraConfig setCameraPitch(double cameraPitch) {
        this.cameraPitch = cameraPitch;
        return this;
    }


    public PhotonCameraConfig setCameraName(String cameraName) {
        this.cameraName = cameraName;
        return this;
    }

    public PhotonCameraConfig setAprilTagPipeline(int aprilTagPipeline) {
        this.aprilTagPipeline = aprilTagPipeline;
        return this;
    }

    public PhotonCameraConfig setConePipeline(int conePipeline) {
        this.conePipeline = conePipeline;
        return this;
    }

    public PhotonCameraConfig setCubePipeline(int cubePipeline) {
        this.cubePipeline = cubePipeline;
        return this;
    }

    public double getTargetHeight() { return targetHeight; }
    public double getCameraHeight() { return cameraHeight; }
    public double getCameraPitch() { return cameraPitch; }
    public String getCameraName() { return cameraName; }
    public int getAprilTagPipeline() { return aprilTagPipeline; }
    public int getConePipeline() { return conePipeline; }
    public int getCubePipeline() { return cubePipeline; }
}
