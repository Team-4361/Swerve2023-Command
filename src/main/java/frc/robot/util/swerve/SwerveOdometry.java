package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.MessageFormat;
import java.util.function.Supplier;

import static frc.robot.Constants.Chassis.ODOMETRY_MS_INTERVAL;

/**
 * the chassis' swerve drive odometry system. this uses encoders on each
 * of the swerve modules, as well as a gyroscope on the robot, to determine
 * the robot's position by integrating its velocity over time. to increase
 * accuracy, this will only update every 5 milliseconds. if it updates too
 * frequently, outlier velocity readings will impact the robot's position and
 * it'll be wrong. if it doesn't update frequently enough, the angle of each
 * of the wheels won't be accounted for properly, which will also make
 * the robot's position wrong
 */
public class SwerveOdometry {

    private final SwerveChassis chassis;
    private final Supplier<Rotation2d> gyroSupplier;
    private final Supplier<SwerveModulePosition[]> positionSupplier;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;
    private double lastUpdateTimeMs;

    public SwerveOdometry(SwerveChassis chassis,
                          Supplier<Rotation2d> gyroSupplier,
                          Supplier<SwerveModulePosition[]> positionSupplier,
                          Pose2d pose) {
        this.chassis = chassis;
        this.gyroSupplier = gyroSupplier;
        this.positionSupplier = positionSupplier;
        this.pose = pose;
        odometry = new SwerveDriveOdometry(
                chassis.getSwerveKinematics(),
                gyroSupplier.get(),
                positionSupplier.get(),
                pose
        );
    }

    private static String formatState(SwerveModuleState state) {
        return MessageFormat.format(
                "v: {1} a: {2} deg",
                state.speedMetersPerSecond,
                state.angle.getDegrees()
        );
    }

    public void update() {
        // each of these states is m per sec and omega rad per sec
        SwerveModuleState frontRightState = chassis.getFrontRight().getState();
        SwerveModuleState frontLeftState = chassis.getFrontLeft().getState();
        SwerveModuleState backRightState = chassis.getBackRight().getState();
        SwerveModuleState backLeftState = chassis.getBackLeft().getState();

        SmartDashboard.putString("FR State", formatState(frontRightState));
        SmartDashboard.putString("FL State", formatState(frontLeftState));
        SmartDashboard.putString("BR State", formatState(backRightState));
        SmartDashboard.putString("BL State", formatState(backLeftState));

        pose = odometry.update(
                gyroSupplier.get(),
                positionSupplier.get()
        );

        lastUpdateTimeMs = System.currentTimeMillis();
    }

    public void reset() {
        odometry.resetPosition(gyroSupplier.get(), positionSupplier.get(), new Pose2d());
    }

    public Pose2d getPose() {
        return pose;
    }

    public boolean shouldUpdate() {
        // only update the odometry every X milliseconds
        // updating it too frequently may cause very inaccurate results
        return System.currentTimeMillis() - ODOMETRY_MS_INTERVAL >= lastUpdateTimeMs;
    }
}