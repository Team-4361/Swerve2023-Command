package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.util.swerve.PWMAbsoluteEncoder;
import swervelib.imu.NavXSwerve;
import swervelib.math.Matter;
import swervelib.motors.SparkMaxSwerve;
import swervelib.parser.*;

import static frc.robot.DriveConstants.Chassis.CHASSIS_SIDE_LENGTH;
import static frc.robot.DriveConstants.PIDConstraint.HEADING_PID;

public class DriveConstants {

    public static class Chassis {
        public static final double ROBOT_MASS = 75; // robot weight in pounds.
        public static final Matter CHASSIS_SIZE = new Matter(new Translation3d(0, 0, Units.inchesToMeters(26)), ROBOT_MASS);
        public static final double LOOP_TIME  = 0.13;
        public static final double CHASSIS_SIDE_LENGTH = 0.6604;

        public static final double CHASSIS_MAX_SPEED = 7; // meters per second.

        public static final boolean GYRO_INVERTED = true;
    }

    public static class Ports {
        public static final int FR_DRIVE_ID = 4;
        public static final int FL_DRIVE_ID = 2;
        public static final int BR_DRIVE_ID = 8;
        public static final int BL_DRIVE_ID = 6;

        public static final int FR_TURN_ID = 3;
        public static final int FL_TURN_ID = 1;
        public static final int BR_TURN_ID = 7;
        public static final int BL_TURN_ID = 5;

        public static final int FR_DIO_ENCODER_PORT = 1;
        public static final int FL_DIO_ENCODER_PORT = 0;
        public static final int BR_DIO_ENCODER_PORT = 3;
        public static final int BL_DIO_ENCODER_PORT = 2;
    }

    public static class Offset {
        // ALL OFFSETS ARE IN DEGREES!
        public static final double FL_OFFSET = 0; //180
        public static final double FR_OFFSET = 220.1; //-226.318;
        public static final double BL_OFFSET = 169.53; //10.07;
        public static final double BR_OFFSET = 193.79; //169.39;
    }

    public static class PIDConstraint {
        public static final PIDFConfig HEADING_PID = new PIDFConfig(0.01, 0, 0, 0);
        public static final PIDFConfig DRIVE_PID = new PIDFConfig(0.01, 0, 0, 0);

        public static final PathConstraints AUTO_CONSTRAINTS = new PathConstraints(3, 3);
    }

    public static class Modules {
        public static final SwerveModulePhysicalCharacteristics SDS_MODULE = new SwerveModulePhysicalCharacteristics(
                8.16,
                12.8,
                5676.0,
                0.10,
                0,
                0,
                1,
                1
        );

        public static final SwerveModuleConfiguration FL_MODULE = new SwerveModuleConfiguration(
                new SparkMaxSwerve(Ports.FL_DRIVE_ID, true),
                new SparkMaxSwerve(Ports.FL_TURN_ID, false),
                new PWMAbsoluteEncoder(Ports.FL_DIO_ENCODER_PORT),
                Offset.FL_OFFSET,
                CHASSIS_SIDE_LENGTH / 2,
                CHASSIS_SIDE_LENGTH / 2,
                PIDConstraint.HEADING_PID,
                PIDConstraint.DRIVE_PID,
                Chassis.CHASSIS_MAX_SPEED,
                Modules.SDS_MODULE,
                false,
                false,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        public static final SwerveModuleConfiguration FR_MODULE = new SwerveModuleConfiguration(
                new SparkMaxSwerve(Ports.FR_DRIVE_ID, true),
                new SparkMaxSwerve(Ports.FR_TURN_ID, false),
                new PWMAbsoluteEncoder(Ports.FR_DIO_ENCODER_PORT),
                Offset.FR_OFFSET,
                Chassis.CHASSIS_SIDE_LENGTH / 2,
                -Chassis.CHASSIS_SIDE_LENGTH / 2,
                PIDConstraint.HEADING_PID,
                PIDConstraint.DRIVE_PID,
                Chassis.CHASSIS_MAX_SPEED,
                Modules.SDS_MODULE,
                false,
                true,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        public static final SwerveModuleConfiguration BL_MODULE = new SwerveModuleConfiguration(
                new SparkMaxSwerve(Ports.BL_DRIVE_ID, true),
                new SparkMaxSwerve(Ports.BL_TURN_ID, false),
                new PWMAbsoluteEncoder(Ports.BL_DIO_ENCODER_PORT),
                Offset.BL_OFFSET,
                -Chassis.CHASSIS_SIDE_LENGTH / 2,
                Chassis.CHASSIS_SIDE_LENGTH / 2,
                PIDConstraint.HEADING_PID,
                PIDConstraint.DRIVE_PID,
                Chassis.CHASSIS_MAX_SPEED,
                Modules.SDS_MODULE,
                false,
                false,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        public static final SwerveModuleConfiguration BR_MODULE = new SwerveModuleConfiguration(
                new SparkMaxSwerve(Ports.BR_DRIVE_ID, true),
                new SparkMaxSwerve(Ports.BR_TURN_ID, false),
                new PWMAbsoluteEncoder(Ports.BR_DIO_ENCODER_PORT),
                Offset.BR_OFFSET,
                -Chassis.CHASSIS_SIDE_LENGTH / 2,
                -Chassis.CHASSIS_SIDE_LENGTH / 2,
                PIDConstraint.HEADING_PID,
                PIDConstraint.DRIVE_PID,
                Chassis.CHASSIS_MAX_SPEED,
                Modules.SDS_MODULE,
                false,
                true,
                false,
                1,
                SDS_MODULE.angleMotorFreeSpeedRPM
        );

        public static final SwerveDriveConfiguration DRIVE_CONFIGURATION = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[]{
                        Modules.FL_MODULE,
                        Modules.FR_MODULE,
                        Modules.BL_MODULE,
                        Modules.BR_MODULE
                },
                new NavXSwerve(SerialPort.Port.kMXP),
                Chassis.CHASSIS_MAX_SPEED,
                Chassis.GYRO_INVERTED
        );

        public static final SwerveControllerConfiguration CONTROLLER_CONFIGURATION = new SwerveControllerConfiguration(
                DRIVE_CONFIGURATION,
                HEADING_PID
        );
    }
}
