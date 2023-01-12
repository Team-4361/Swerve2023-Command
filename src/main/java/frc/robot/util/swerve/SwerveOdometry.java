package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import me.wobblyyyy.pathfinder2.geometry.PointXYZ;
import me.wobblyyyy.pathfinder2.robot.AbstractOdometry;
import me.wobblyyyy.pathfinder2.time.Time;
import me.wobblyyyy.pathfinder2.utils.StringUtils;
import me.wobblyyyy.pathfinder2.wpilib.WPIAdapter;

import java.util.function.Supplier;

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
public class SwerveOdometry extends AbstractOdometry {
    private final SwerveChassis chassis;
    private final Supplier<Rotation2d> gyroSupplier;
    private final Supplier<SwerveModulePosition[]> positionSupplier;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;
    private double lastUpdateTimeMs;

    /**
     * Creates a new {@link SwerveOdometry} system, combined with the supplied values.
     *
     * @param chassis The {@link SwerveChassis} to be used for the {@link SwerveDriveKinematics}.
     * @param gyroSupplier The {@link Supplier<Rotation2d>} to be used for the Gyroscope.
     */
    public SwerveOdometry(SwerveChassis chassis, Supplier<Rotation2d> gyroSupplier, Supplier<SwerveModulePosition[]> positionSupplier) {
        this.chassis = chassis;
        this.gyroSupplier = gyroSupplier;
        this.positionSupplier = positionSupplier;

        odometry = new SwerveDriveOdometry(
                chassis.getSwerveKinematics(),
                gyroSupplier.get(),
                positionSupplier.get(),
                pose
        );
    }

    /**
     * Formats the {@link SwerveModuleState} into readable values, containing m/s velocity and wheel rotation
     * in degrees.
     *
     * @param state The {@link SwerveModuleState} to use for readings.
     * @return A parsed {@link String} containing m/s and wheel rotation. (v: () a: () deg)
     */
    private static String formatState(SwerveModuleState state) {
        return StringUtils.format(
                "v: %s a: %s deg",
                state.speedMetersPerSecond,
                state.angle.getDegrees()
        );
    }

    /**
     * @return The {@link Rotation2d} drive heading reported by the {@link SwerveOdometry}. This should be the
     * only Gyro reading <b>robot-wide</b>, other than reading from the reported {@link Pose2d}.
     */
    public Rotation2d getDriveHeading() {
        return pose.getRotation();
    }

    /**
     * Updates the {@link SwerveOdometry} using each individual {@link SwerveModuleState}, adding the values to
     * the {@link SmartDashboard}, and resetting the time that was last updated.
     */
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

        lastUpdateTimeMs = Time.ms();
    }

    /**
     * Resets the Odometry position, using a default {@link Pose2d}, and the Gyro supplier. Note that the Gyroscope
     * <b>should not be reset</b>, as the Gyroscope reading in the Odometry would be offset (becomes 0).
     */
    public void reset() {
        odometry.resetPosition(gyroSupplier.get(), positionSupplier.get(), pose);
    }

    /** @return The reported {@link Pose2d} from the {@link SwerveOdometry} */
    public Pose2d getPose() {
        return pose;
    }

    /** @return If the last updated time was greater than {@link Constants.Chassis#ODOMETRY_MS_INTERVAL} */
    public boolean shouldUpdate() {
        // only update the odometry every X milliseconds
        // updating it too frequently may cause very inaccurate results
        double updateInterval = Constants.Chassis.ODOMETRY_MS_INTERVAL;
        return Time.ms() - updateInterval >= lastUpdateTimeMs;
    }

    @Override
    public PointXYZ getRawPosition() {
        return new PointXYZ();
    }
}
