package frc.robot.util.math;

public enum CameraQuality {

    VERY_FAST(256, 144),
    FAST(320, 240),
    REGULAR(640, 480),
    QUALITY(960, 720),
    HIGH_QUALITY(1280, 720);

    public final int width, height;

    CameraQuality(int width, int height) {
        this.width = width;
        this.height = height;
    }
}
