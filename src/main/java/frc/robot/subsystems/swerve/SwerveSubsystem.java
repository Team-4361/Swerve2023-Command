// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.DriveConstants;
import frc.robot.Robot;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.telemetry.SwerveDriveTelemetry;

import static frc.robot.DriveConstants.Modules.CONTROLLER_CONFIGURATION;
import static frc.robot.DriveConstants.Modules.DRIVE_CONFIGURATION;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    private boolean fieldOriented = true, openLoop = true, precisionMode = false;

    public boolean isFieldOriented() {
        return fieldOriented;
    }

    public boolean isPrecisionMode() {
        return precisionMode;
    }

    public SwerveSubsystem setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
        return this;
    }

    public SwerveSubsystem setPrecisionMode(boolean precision) {
        this.precisionMode = precision;
        return this;
    }

    public boolean isOpenLoop() {
        return openLoop;
    }

    public SwerveSubsystem setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
        return this;
    }

    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }
    public Command toggleOpenLoopCommand() { return Commands.runOnce(() -> openLoop = !openLoop); }
    public Command lockWheelCommand() { return this.runOnce(swerveDrive::lockPose); }

    public Command resetGyroCommand() { return this.runOnce(swerveDrive::zeroGyro); }

    public Command holdPrecisionModeCommand() {
        return Commands.runEnd(
                () -> { precisionMode = true; },
                () -> { precisionMode = false; }
        );
    }

    /**
     * Construct the swerve drive.
     */
    public SwerveSubsystem() {
        swerveDrive = new SwerveDrive(DRIVE_CONFIGURATION, CONTROLLER_CONFIGURATION);

        swerveDrive.setMotorIdleMode(true); // brake mode

        SwerveDriveTelemetry.verbosity = HIGH;
    }

    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     * @param isOpenLoop    Whether to use closed-loop velocity control.  Set to true to disable closed-loop.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        SwerveDriveTelemetry.updateData();

        SmartDashboard.putData("Field", swerveDrive.field);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveKinematics2} of the swerve drive.
     */
    public SwerveKinematics2 getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Resets the gyro angle to zero and resets Odometry to the same position, but facing toward 0.
     */
    public void resetGyro() {
        swerveDrive.zeroGyro();
    }


    /**
     * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), false, 1);
    }
}
