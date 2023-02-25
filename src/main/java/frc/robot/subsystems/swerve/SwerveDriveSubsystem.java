package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.*;
import swervelib.parser.SwerveControllerConfiguration;

import static frc.robot.Constants.Chassis.*;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveDrive}, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveChassis;

    private boolean fieldOriented = true, openLoop = true;

    /*
    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                swerveChassis::getPose,
                swerveChassis::getSwerveModulePoses,
                new PIDController(0,0,0),
                new PIDController(0,0,0),
                new PIDController(0,0,0),
                swerveChassis::setModuleStates
        );
    }
     */

    public Command toggleFieldOriented() {
        return this.runOnce(() -> fieldOriented = !fieldOriented).andThen(resetGyroCommand());
    }

    public Command toggleOpenLoop() {
        return this.runOnce(() -> openLoop = !openLoop);
    }

    public Command resetGyroCommand() {
        return this.runOnce(this::resetPosition);
    }

    /** Initializes a new {@link SwerveDriveSubsystem}, and resets the Gyroscope. */
    public SwerveDriveSubsystem() {
        swerveChassis = new SwerveDrive(
                DRIVE_CONFIGURATION,
                new SwerveControllerConfiguration(
                        DRIVE_CONFIGURATION,
                        HEADING_PID
                )
        );

        swerveChassis.setMotorIdleMode(true); // enable brake mode
        swerveChassis.zeroGyro(); // zero gyroscope


        // Don't run the PathPlannerServer during a competition to save bandwidth.
        if (!DriverStation.isFMSAttached())
            PathPlannerServer.startServer(5811);
    }

    /**
     * @return A {@link Rotation2d} containing the current rotation of the robot
     */
    public Rotation2d getRobotHeading() {
        return swerveChassis.getYaw();
    }

    @Override
    public void periodic() {
        // Update the robot speed and other information.

        swerveChassis.updateOdometry();

        SmartDashboard.putString("Robot Position", swerveChassis.getPose().toString());
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
        swerveChassis.setChassisSpeeds(speeds);
    }

    /**
     * Drives the Robot without field orientation, made for autonomous use with closed-loop control.
     *
     * @param vX X-direction m/s (+ forward, - reverse)
     * @param vY Y-direction m/s (+ left, - right)
     * @param omega Yaw rad/s (+ left, - right)
     */
    public void autoDrive(double vX, double vY, double omega) {
        swerveChassis.drive(new Translation2d(vX, vY), omega, false, false);
    }


    /**
     * Drives the Robot using specific speeds, which is converted to field-relative or robot-relative
     * automatically. Unlike {@link #drive(ChassisSpeeds)}, it will compensate for the angle of the Robot.
     *
     * @param vX X-direction m/s (+ forward, - reverse)
     * @param vY Y-direction m/s (+ left, - right)
     * @param omega Yaw rad/s (+ left, - right)
     */
    public void teleopDrive(double vX, double vY, double omega) {
        swerveChassis.drive(new Translation2d(vX, vY), omega, fieldOriented, openLoop);
    }

    public void driveForward() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, Rotation2d.fromDegrees(0))); }
    public void driveBack() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(-0.8, 0, 0, Rotation2d.fromDegrees(0))); }
    public void driveLeft() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, Rotation2d.fromDegrees(0))); }
    public void driveRight() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, -0.8, 0, Rotation2d.fromDegrees(0))); }
    public void stop() { drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0))); }

    public static double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }

    /**
     * Resets the Odometry, which will offset the gyro angle and cause it to become zero while
     * being referenced, essentially resetting the gyroscope.
     */
    public void resetPosition() {
        swerveChassis.resetOdometry(new Pose2d());
    }

    /** @return The currently used {@link SwerveDrive} */
    public SwerveDrive getSwerveChassis(){
        return swerveChassis;
    }
}