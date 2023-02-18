package frc.robot.util.math;

public class GearRatio {
    public static double degreesToMotorRotations(double degrees, double ratio) {
        return (ratio / 360) * degrees;
    }

    public static double motorRotationsToDegrees(double rotations, double ratio) {
        return (360 / ratio) * rotations;
    }
}
