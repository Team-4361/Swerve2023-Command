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
        public static final double BR_OFFSET = -193.79; //169.39;
    }

    public static class PIDConstraint {
        public static final PIDFConfig HEADING_PID = new PIDFConfig(0.01, 0, 0, 0);
        public static final PIDFConfig DRIVE_PID = new PIDFConfig(0.01, 0, 0, 0);

        public static final PathConstraints AUTO_CONSTRAINTS = new PathConstraints(3, 3);
    }

    public static class Modules {

    }
}
