package frc.robot.util.math;

public class PeakMotorDistance {
    private final Distance maxDistance;
    private final DistanceUnit maxDistanceUnit;
    private final double maxRotation;

    public PeakMotorDistance(Distance distance, DistanceUnit unit, double rotation) {
        assert rotation != 0;
        this.maxDistance = distance;
        this.maxDistanceUnit = unit;
        this.maxRotation = rotation;
    }

    public Distance getDistance() {
        return maxDistance;
    }

    public double getRotation() {
        return maxRotation;
    }

    public DistanceUnit getDistanceUnit() {
        return maxDistanceUnit;
    }

    /** @return rotation to distance based on unit */
    public double rotationToDistance(double currentRotation) {
        return (currentRotation / maxRotation) * maxDistance.getValue(maxDistanceUnit);
    }

    public double distanceToRotation(double currentDistance) {
        return (currentDistance / maxDistance.getValue(maxDistanceUnit)) * maxRotation;
    }
}
