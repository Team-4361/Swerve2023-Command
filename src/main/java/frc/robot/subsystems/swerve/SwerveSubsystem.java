// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.swerve.PWMAbsoluteEncoder;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.imu.NavXSwerve;
import swervelib.math.SwerveKinematics2;
import swervelib.motors.SparkMaxSwerve;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.telemetry.SwerveDriveTelemetry;

import static frc.robot.DriveConstants.Chassis.CHASSIS_SIDE_LENGTH;
import static frc.robot.DriveConstants.PIDConstraint.HEADING_PID;
import static frc.robot.DriveConstants.Ports.*;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.LOW;

public class SwerveSubsystem extends SubsystemBase {

    private SwerveDrive swerveDrive;

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
    private SwerveModulePhysicalCharacteristics SDS_MODULE;
    private SwerveModuleConfiguration FL_MODULE, FR_MODULE, BL_MODULE, BR_MODULE;
    private SwerveDriveConfiguration DRIVE_CONFIGURATION;
    private SwerveControllerConfiguration CONTROLLER_CONFIGURATION;

    private final SparkMaxSwerve flDrive, frDrive, blDrive, brDrive;
    private final SparkMaxSwerve flTurn, frTurn, blTurn, brTurn;

    public SwerveSubsystem() {
        flDrive = new SparkMaxSwerve(DriveConstants.Ports.FL_DRIVE_ID, true);
        frDrive = new SparkMaxSwerve(DriveConstants.Ports.FR_DRIVE_ID, true);
        blDrive = new SparkMaxSwerve(DriveConstants.Ports.BL_DRIVE_ID, true);
        brDrive = new SparkMaxSwerve(DriveConstants.Ports.BR_DRIVE_ID, true);

        flTurn = new SparkMaxSwerve(FL_TURN_ID, false);
        frTurn = new SparkMaxSwerve(FR_TURN_ID, false);
        blTurn = new SparkMaxSwerve(BL_TURN_ID, false);
        brTurn = new SparkMaxSwerve(BR_TURN_ID, false);

        SDS_MODULE = new SwerveModulePhysicalCharacteristics(
                8.16,
                12.8,
                5676.0,
                0.10,
                0,
                0,
                1,
                1
        );

        FL_MODULE = new SwerveModuleConfiguration(
                flDrive,
                flTurn,
                new PWMAbsoluteEncoder(DriveConstants.Ports.FL_DIO_ENCODER_PORT),
                DriveConstants.Offset.FL_OFFSET,
                CHASSIS_SIDE_LENGTH / 2,
                CHASSIS_SIDE_LENGTH / 2,
                DriveConstants.PIDConstraint.HEADING_PID,
                DriveConstants.PIDConstraint.DRIVE_PID,
                DriveConstants.Chassis.CHASSIS_MAX_SPEED,
                SDS_MODULE,
                false,
                false,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        FR_MODULE = new SwerveModuleConfiguration(
                frDrive,
                frTurn,
                new PWMAbsoluteEncoder(DriveConstants.Ports.FR_DIO_ENCODER_PORT),
                DriveConstants.Offset.FR_OFFSET,
                DriveConstants.Chassis.CHASSIS_SIDE_LENGTH / 2,
                -DriveConstants.Chassis.CHASSIS_SIDE_LENGTH / 2,
                DriveConstants.PIDConstraint.HEADING_PID,
                DriveConstants.PIDConstraint.DRIVE_PID,
                DriveConstants.Chassis.CHASSIS_MAX_SPEED,
                SDS_MODULE,
                false,
                true,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        BL_MODULE = new SwerveModuleConfiguration(
                blDrive,
                blTurn,
                new PWMAbsoluteEncoder(DriveConstants.Ports.BL_DIO_ENCODER_PORT),
                DriveConstants.Offset.BL_OFFSET,
                -DriveConstants.Chassis.CHASSIS_SIDE_LENGTH / 2,
                DriveConstants.Chassis.CHASSIS_SIDE_LENGTH / 2,
                DriveConstants.PIDConstraint.HEADING_PID,
                DriveConstants.PIDConstraint.DRIVE_PID,
                DriveConstants.Chassis.CHASSIS_MAX_SPEED,
                SDS_MODULE,
                false,
                false,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        BR_MODULE = new SwerveModuleConfiguration(
                brDrive,
                brTurn,
                new PWMAbsoluteEncoder(DriveConstants.Ports.BR_DIO_ENCODER_PORT),
                DriveConstants.Offset.BR_OFFSET,
                -DriveConstants.Chassis.CHASSIS_SIDE_LENGTH / 2,
                -DriveConstants.Chassis.CHASSIS_SIDE_LENGTH / 2,
                DriveConstants.PIDConstraint.HEADING_PID,
                DriveConstants.PIDConstraint.DRIVE_PID,
                DriveConstants.Chassis.CHASSIS_MAX_SPEED,
                SDS_MODULE,
                false,
                true,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        DRIVE_CONFIGURATION = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[]{
                        FL_MODULE,
                        FR_MODULE,
                        BL_MODULE,
                        BR_MODULE
                },
                new NavXSwerve(SerialPort.Port.kMXP),
                DriveConstants.Chassis.CHASSIS_MAX_SPEED,
                DriveConstants.Chassis.GYRO_INVERTED
        );

        CONTROLLER_CONFIGURATION = new SwerveControllerConfiguration(
                DRIVE_CONFIGURATION,
                HEADING_PID
        );

        swerveDrive = new SwerveDrive(DRIVE_CONFIGURATION, CONTROLLER_CONFIGURATION);

        SwerveDriveTelemetry.verbosity = LOW;
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
        SmartDashboard.putString("Robot Position", swerveDrive.getPose().toString());
        SmartDashboard.putString("Robot Angle", swerveDrive.getYaw().toString());
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
