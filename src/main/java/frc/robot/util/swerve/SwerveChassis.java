package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.Chassis.*;

public class SwerveChassis {
    private static final Translation2d SWERVE_FL_POSITION =
            new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final Translation2d SWERVE_FR_POSITION =
            new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final Translation2d SWERVE_BL_POSITION =
            new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final Translation2d SWERVE_BR_POSITION =
            new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(
                    SWERVE_FL_POSITION,
                    SWERVE_FR_POSITION,
                    SWERVE_BL_POSITION,
                    SWERVE_BR_POSITION
            );

    private static double average(double... vals)  {
        return Arrays.stream(vals).sum() / vals.length;
    }

    private static final String NAME_FL = "FL";
    private static final String NAME_FR = "FR";
    private static final String NAME_BL = "BL";
    private static final String NAME_BR = "BR";

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    public SwerveChassis() {
        this(
                new SwerveModule(
                        FL_DRIVE_ID,
                        FL_TURN_ID,
                        FL_DIO_ENCODER_PORT,
                        FL_OFFSET,
                        FL_ERROR_FACTOR
                ),
                new SwerveModule(
                        FR_DRIVE_ID,
                        FR_TURN_ID,
                        FR_DIO_ENCODER_PORT,
                        FR_OFFSET,
                        FR_ERROR_FACTOR
                ),
                new SwerveModule(
                        BL_DRIVE_ID,
                        BL_TURN_ID,
                        BL_DIO_ENCODER_PORT,
                        BL_OFFSET,
                        BL_ERROR_FACTOR
                ),
                new SwerveModule(
                        BR_DRIVE_ID,
                        BR_TURN_ID,
                        BR_DIO_ENCODER_PORT,
                        BR_OFFSET,
                        BR_ERROR_FACTOR
                )
        );
        updateDashboard();
    }

    public SwerveChassis(SwerveModule frontLeft,
                         SwerveModule frontRight,
                         SwerveModule backLeft,
                         SwerveModule backRight) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        updateDashboard();
    }

    private void updateDashboard() {
        frontLeft.updateDashboard(NAME_FL);
        frontRight.updateDashboard(NAME_FR);
        backLeft.updateDashboard(NAME_BL);
        backRight.updateDashboard(NAME_BR);
    }

    public SwerveModule getFrontLeft() {
        return frontLeft;
    }

    public SwerveModule getFrontRight() {
        return frontRight;
    }

    public SwerveModule getBackLeft() {
        return backLeft;
    }


    public SwerveModule getBackRight() {
        return backRight;
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return SWERVE_KINEMATICS;
    }

    public HashMap<String, SwerveModuleState> getSwerveModuleStates() {
        return new HashMap<>(Map.of(
                "FL", getFrontLeft().getState(),
                "BL", getBackLeft().getState(),
                "FR", getFrontRight().getState(),
                "BR", getBackRight().getState()
        ));
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                getFrontLeft().getPosition(),
                getFrontRight().getPosition(),
                getBackLeft().getPosition(),
                getBackRight().getPosition()
        };
    }

    public void setStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        setStates(SWERVE_KINEMATICS.toSwerveModuleStates(speeds));
        updateDashboard();
    }

    public void resetDriveEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }

    public SwerveChassis getSwerveChassis(){
        return this;
    }
}