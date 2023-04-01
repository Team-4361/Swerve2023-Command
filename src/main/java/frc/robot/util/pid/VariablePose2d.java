package frc.robot.util.pid;

import edu.wpi.first.math.geometry.Pose2d;

public class VariablePose2d {
    private double x, y, degrees;

    public VariablePose2d(double x, double y, double degrees) {
        this.x = x;
        this.y = y;
        this.degrees = degrees;
    }

    public static VariablePose2d fromPose(Pose2d pose) {
        return new VariablePose2d(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public VariablePose2d(double x, double y) { this(x, y, Double.MIN_VALUE); }

    public VariablePose2d x(double x) { this.x = x; return this; }
    public VariablePose2d y(double y) { this.y = y; return this; }

    public VariablePose2d deg(double degrees) { this.degrees = degrees; return this; }
    public VariablePose2d rad(double radians) { this.degrees = Math.toDegrees(radians); return this; }

    public boolean hasX() { return x != Double.MIN_VALUE; }
    public boolean hasY() { return y != Double.MIN_VALUE; }
    public boolean hasRotation() { return degrees != Double.MIN_VALUE; }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getDegrees() { return degrees; }
    public double getRadians() { return Math.toRadians(degrees); }
}
