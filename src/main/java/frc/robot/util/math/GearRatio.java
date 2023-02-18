package frc.robot.util.math;

public class GearRatio {

    private double ratio;
    public GearRatio(double ratio) {
        this.ratio = ratio;
    }

    public double degreesToMotorRotations(double degrees) {
        return (ratio / 360) * degrees;
    }
    public double motorRotationsToDegrees(double rotations) {
        return (360 / ratio) * rotations;
    }

    public static GearRatio fromRatio(double ratio) {
        return new GearRatio(ratio);
    }

    public static double degreesToMotorRotations(double degrees, double ratio) {
        return (ratio / 360) * degrees;
    }

    public static double motorRotationsToDegrees(double rotations, double ratio) {
        return (360 / ratio) * rotations;
    }
}
