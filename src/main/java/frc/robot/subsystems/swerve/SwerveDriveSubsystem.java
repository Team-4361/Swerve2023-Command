package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.swerve.SwerveChassis;
import frc.robot.util.swerve.SwerveModule;
import frc.robot.util.swerve.SwerveOdometry;

import java.util.HashMap;

import static frc.robot.Constants.TEST_MODE;

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

    public static boolean fieldOriented = true, precisionMode = false, closedLoop = false;

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

    public Command followTrajectoryCommand(String fileName) {
        return followTrajectoryCommand(PathPlanner.loadPath(fileName, new PathConstraints(3, 3)));
    }

    public Command holdPrecisionModeCommand() {
        return Commands.runEnd(() -> precisionMode = true, () -> precisionMode = false);
    }

    public Pose2d getPose() {
        return odometry.getPose();
    }

    public Command lockWheelCommand() {
        return Commands.run(() -> {
            swerveChassis.setStates(
                    new SwerveModuleState[]{
                            new SwerveModuleState(0, new Rotation2d(Math.PI/2)),
                            new SwerveModuleState(0, new Rotation2d(Math.PI/2)),
                            new SwerveModuleState(0, new Rotation2d(Math.PI/2)),
                            new SwerveModuleState(0, new Rotation2d(Math.PI/2))
                    }
            );
        });
    }

    public Command toggleFieldOrientedCommand() {
        return this.runOnce(() -> {
            fieldOriented = !fieldOriented;
        }).andThen(resetGyroCommand());
    }

    public Command toggleClosedLoopCommand() {
        return this.runOnce(() -> {
            closedLoop = !closedLoop;
        });
    }

    public Command resetGyroCommand() {
        return this.runOnce(this::resetPosition);
    }

    /** Initializes a new {@link SwerveDriveSubsystem}, and resets the Gyroscope. */
    public SwerveDriveSubsystem(SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br, double sideLength) {
        swerveChassis = new SwerveChassis(fl, fr, bl, br, sideLength);
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
     * @return A {@link Rotation2d} containing the current rotation of the robot
     */
    public Rotation2d getRobotHeading() {
        return robotHeading;
    }

    @Override
    public void periodic() {
        // Update the robot speed and other information.
        robotHeading = new Rotation2d(gyro.getRotation2d().getRadians());

        if (odometry.shouldUpdate())
            odometry.update();

        SmartDashboard.putNumber("Robot Angle", robotHeading.getDegrees());

        if (TEST_MODE) {
            SmartDashboard.putString("Robot Position", odometry.getPose().toString());
            SmartDashboard.putBoolean("Gyro Calibrating", gyro.isCalibrating());
        }
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
     * automatically. Unlike {@link #drive(ChassisSpeeds)}, it will compensate for the angle of the Robot.
     *
     * @param vX X-direction m/s (+ forward, - reverse)
     * @param vY Y-direction m/s (+ left, - right)
     * @param omega Yaw rad/s (+ left, - right)
     */
    public void autoDrive(double vX, double vY, double omega) {
        if (precisionMode) {
            vX /= 3;
            vY /= 3;
            omega /= 3;
        }
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-vX, -vY, omega, fieldOriented ? odometry.getPose().getRotation() : new Rotation2d(0)));
    }

    /**
     * Drives the Robot using specific speeds and a manually
     * specified Robot "heading".
     * Unlike {@link #drive(ChassisSpeeds)}, it will compensate for the angle of the Robot, and
     * adds flips the value of {@code vY} and {@code omega} to convert into "robot geometry"
     *
     * @param vX X-direction m/s (+ forward, - reverse)
     * @param vY Y-direction m/s (+ left, - right)
     * @param omega Yaw rad/s (+ left, - right)
     */
    public void robotDrive(double vX, double vY, double omega, double heading) {
        if (precisionMode) {
            vX /= 3;
            vY /= 3;
            omega /= 3;
        }
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-vX, -vY, omega, new Rotation2d(heading)));
    }

    public void driveForward() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0))); }
    public void driveBack() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.5, 0, 0, Rotation2d.fromDegrees(0))); }
    public void driveLeft() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0))); }
    public void driveRight() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0))); }
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