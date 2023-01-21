package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.swerve.SwerveChassis;
import frc.robot.util.swerve.SwerveOdometry;

import java.util.HashMap;
import java.util.function.Supplier;

import static frc.robot.Constants.AutoValues.*;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveChassis}, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    public final AHRS gyro;
    private final SwerveChassis swerveChassis;
    private final SwerveOdometry odometry;
    private Rotation2d robotHeading;
    private final PIDController controller;

    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                odometry::getPose,
                swerveChassis.getSwerveKinematics(),
                new PIDController(0,0,0),
                new PIDController(0,0,0),
                new PIDController(0,0,0),
                swerveChassis::setStates
        );
    }

    public Command resetGyroCommand() {
        return this.runOnce(this::resetPosition);
    }

    /** Initializes a new {@link SwerveDriveSubsystem}, and resets the Gyroscope. */
    public SwerveDriveSubsystem() {
        swerveChassis = new SwerveChassis();
        controller = new PIDController(PID_PROPORTIONAL, PID_INTEGRAL, PID_DERIVATIVE);
        gyro = new AHRS(SPI.Port.kMXP);
        robotHeading = new Rotation2d(0);

        // Don't run the PathPlannerServer during a competition to save bandwidth.
        if (!DriverStation.isFMSAttached())
            PathPlannerServer.startServer(5811);

        odometry = new SwerveOdometry(
                swerveChassis,
                this::getRobotHeading,
                swerveChassis::getSwerveModulePositions,
                new Pose2d()
        );

        gyro.reset();
        gyro.calibrate();

        resetPosition();
    }

    /**
     * Checks if the actual value is within a specified tolerance of the expected value
     * @param expected The value to be expected.
     * @param actual The actual value.
     * @param tolerance The maximum error or tolerance that the value can be offset to still be true.
     * @return True/false depending on tolerance.
     */
    private static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    public void driveToPose(Pose2d currentPose, Pose2d desiredPose) {
        robotDrive(
                MathUtil.clamp(controller.calculate(currentPose.getX(), desiredPose.getX()), -0.5, 0.5),
                -MathUtil.clamp(controller.calculate(currentPose.getY(), desiredPose.getY()), -0.5, 0.5),
                MathUtil.clamp(controller.calculate(currentPose.getRotation().getDegrees(), 180), -0.4, 0.4),
                0 // 0 degree heading is used to disable field-relative temporarily
        );
    }

    public static boolean isCorrectPose(Pose2d currentPose, Pose2d desiredPose) {
        return (
                inTolerance(currentPose.getX(), desiredPose.getX(), 0.5) &&
                inTolerance(currentPose.getY(), desiredPose.getY(), 0.5) &&
                inTolerance(currentPose.getRotation().getDegrees(), 180, 4)
        );
    }

    /**
     * @return A {@link Rotation2d} containing the current rotation of the robot
     */
    public Rotation2d getRobotHeading() {
        return robotHeading;
    }

    @Override
    public void periodic() {
        // Update the robot speed and other information.
        robotHeading = gyro.getRotation2d();

        if (odometry.shouldUpdate()) {
            odometry.update();
        }

        SmartDashboard.putNumber("Robot MPH", swerveChassis.getDriveMPH());
        SmartDashboard.putNumber("Robot Max MPH", swerveChassis.getMaxDriveMPH());
        SmartDashboard.putString("Robot Actual Heading", robotHeading.toString());
        SmartDashboard.putString("Robot Position", odometry.getPose().toString());
        SmartDashboard.putBoolean("Gyro Calibrating", gyro.isCalibrating());
    }

    /** @return A {@link HashMap} containing {@link SwerveModuleState} of the robot. */
    public HashMap<String, SwerveModuleState> getSwerveModuleStates() {
        return swerveChassis.getSwerveModuleStates();
    }

    /**
     * Manually drives the robot in a specific direction, using raw {@link ChassisSpeeds}. This is not recommended
     * because it can override the desired driving mode (robot-relative/field-relative) which autoDrive will
     * compensate for. Held <b>indefinitely</b> until this method is recalled again.
     *
     * @param speeds The {@link ChassisSpeeds} to drive the robot with.
     * @see ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)
     */
    public void drive(ChassisSpeeds speeds) {
        swerveChassis.drive(speeds);
    }

    /**
     * Drives the Robot using specific speeds, which is converted to field-relative or robot-relative
     * automatically. Unlike {@link #drive(ChassisSpeeds)}, it will compensate for the angle of the Robot, and
     * adds flips the value of {@code vY} and {@code omega} to convert into "robot geometry"
     *
     * @param vX X-direction m/s (+ right, - left)
     * @param vY Y-direction m/s (+ forward, - reverse)
     * @param omega Yaw rad/s (+ right, - left)
     */
    public void autoDrive(double vX, double vY, double omega) {
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vX, -vY, -omega, odometry.getPose().getRotation()));
    }

    /**
     * Drives the Robot using specific speeds and a manually
     * specified Robot "heading".
     * Unlike {@link #drive(ChassisSpeeds)}, it will compensate for the angle of the Robot, and
     * adds flips the value of {@code vY} and {@code omega} to convert into "robot geometry"
     *
     * @param vX X-direction m/s (+ right, - left)
     * @param vY Y-direction m/s (+ forward, - reverse)
     * @param omega Yaw rad/s (+ right, - left)
     */
    public void robotDrive(double vX, double vY, double omega, double heading) {
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vX, -vY, -omega, new Rotation2d(heading)));
    }

    /** Drives the robot to the right direction at 0.8 m/s (possibly?) */
    public void driveRight() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0))); }

    /** Drives the robot to the left direction at 0.8 m/s (possibly?) */
    public void driveLeft() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, 0, Rotation2d.fromDegrees(0))); }

    /** Drives the robot forward at 0.8 m/s (possibly?) */
    public void driveForward() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0))); }

    /** Drives the robot backwards at 0.8 m/s (possibly?) */
    public void driveBack() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0))); }

    /** Stops the robot from moving completely, will usually not release brake mode from testing. */
    public void stop() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0))); }

    /** @return {@link Rotation2d} gyroscope instance. */
    public Rotation2d getGyro() {
        return robotHeading;
    }

    public static double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }

    /**
     * Resets the Odometry, which will offset the gyro angle and cause it to become zero while
     * being referenced, essentially resetting the gyroscope.
     */
    public void resetPosition() {
        odometry.resetOdometry();
    }

    /** @return The currently used {@link SwerveChassis} */
    public SwerveChassis getSwerveChassis(){
        return swerveChassis;
    }
}