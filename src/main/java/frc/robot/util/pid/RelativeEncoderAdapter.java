package frc.robot.util.pid;

import com.revrobotics.RelativeEncoder;

public class RelativeEncoderAdapter {
    private final RelativeEncoder encoder;
    private double distance;

    private long lastUpdatedMillis = 0;

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getPosition() {
        return distance;
    }

    public void setPosition(double position) {
        this.distance = position;
    }

    public RelativeEncoderAdapter(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    private static double calculateRotationChange(double encoderRpm, double elapsedTimeMs) {
        return encoderRpm / (60 / (elapsedTimeMs / 1_000));
    }

    public void update() {
        distance += calculateRotationChange(getVelocity(), System.currentTimeMillis()-lastUpdatedMillis);
        lastUpdatedMillis = System.currentTimeMillis();
    }
}
