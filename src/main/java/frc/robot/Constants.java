// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.camera.PhotonCameraConfig;
import frc.robot.util.math.Distance;
import frc.robot.util.math.DistanceUnit;
import frc.robot.util.math.GearRatio;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.pid.*;
import frc.robot.util.swerve.SwerveModule;

import java.util.HashMap;
import java.util.Map;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.util.math.DistanceUnit.INCHES;
import static java.util.Map.entry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean TEST_MODE = false;

    /**
     * This {@link FrontCamera} class is designed to contain the required values for the PhotonVision
     * AprilTag tracking system.
     */
    public static class FrontCamera {

        public static final int TAG_PIPELINE = 0;
        public static final int CUBE_PIPELINE = 1;
        public static final int CONE_PIPELINE = 2;
        public static final int DRIVER_PIPELINE = 3;

        public static final PhotonCameraConfig CAMERA_CONFIG = new PhotonCameraConfig()
                .setCameraName("frontcamera")
                .setCameraHeight(0.45)
                .setTargetHeight(0)
                .setAprilTagPipeline(1)
                .setConePipeline(2)
                .setCubePipeline(3);

    }

    public static class VacuumValues {
        public static int[] VACUUM_MOTOR_ID = new int[]{20, 16, 13, 11};
        public static double VACUUM_PUMP_SPEED = 0.45;
        public static MotorType VACUUM_MOTOR_TYPE = kBrushed;
        public static int VACUUM_SOLENOID_ONE = 7;
        public static int VACUUM_SOLENOID_TWO = 3;
        public static int VACUUM_SOLENOID_THREE = 1;
        
        public static int VACUUM_SOLENOID_FOUR = 2;

        public static int VACUUM_SENSOR_ONE = 0;
        public static int VACUUM_SENSOR_TWO = 1;
        public static int VACUUM_SENSOR_THREE = 2;
        public static int VACUUM_SENSOR_FOUR = 3;

        public static double VACUUM_THRESHOLD = 1;

    }

    public static class ClimberWristValues {
        public static final int WRIST_GEAR_RATIO = 100;
        public static final int WRIST_MOTOR_ID = 22; // TODO: change!

        public static final double[] WRIST_ANGLE_PRESETS = new double[]{
                5,
                10,
                15,
                20
        };
    }

    public static class ClimberArmValues {
        public static final int ROTATION_MOTOR_ID = 10;
        public static final int EXTENSION_MOTOR_ID = 21;
        //public static final GearRatio ROTATION_GEAR_RATIO = GearRatio.fromRatio(1029); OLD
        public static final GearRatio ROTATION_GEAR_RATIO = GearRatio.fromRatio(735);

        public static final PeakMotorDistance EXTENSION_LIMIT = new PeakMotorDistance(
                Distance.fromValue(50.5, INCHES),
                DistanceUnit.INCHES,
                88
        );

        public static final double WRIST_ROLLOVER_VALUE = 13180.0;
    }

    public static class ClimberPresets {
        //public static final String ROTATION_NAME = "Climber Rotation";
        //public static final String EXTENSION_NAME = "Climber Extension";
        //public static final String WRIST_NAME = "Climber Wrist";
        public static final String ROTATION_NAME = "CLI ROT";
        public static final String EXTENSION_NAME = "CLI EXT";
        public static final String WRIST_NAME = "CLIM WST";

        /* OLD
        // PRESET 0 = ZERO POSITION
        // PRESET 1 = AUTO CONE FLOOR (NO TELEOP)
        // PRESET 2 = HUMAN STATION PICKUP
        // PRESET 3 = CONE DROP OFF
        // PRESET 4 = CONE FLOOR POSITION
        // PRESET 5 = CUBE FLOOR POSITION
        // PRESET 6 = CUBE HIGH POSITION
        // PRESET 7 = MID CONE
        // PRESET 8 = HIGH CONE
        // PRESET 9 = CONE FLOOR


        public static final PresetList ROTATION_PRESETS = new PresetList(0.0, 30.0, -47.987, -62.0, -124.0, -105.0, -65.0, -65.03, -59.0, -135.616);
        public static final PresetList EXTENSION_PRESETS = new PresetList(0.0, 0.0, 30.261, 27.5, 49.0, 25.0, 88.35, 67.049, 86.7, 10.619);

        //public static final PresetList ROTATION_PRESETS = new PresetList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        //public static final PresetList EXTENSION_PRESETS = new PresetList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         */

        ///////////////////////////////////////////////////////////// NEW WARREN HILLS (3/24/23)


        // PRESET 0 = ZERO POSITION         (ZERO_POSITION_INDEX)
        // PRESET 1 = HUMAN STATION PICKUP  (HUMAN_STATION_INDEX)
        // PRESET 2 = FLOOR CONE            (FLOOR_CONE_INDEX)
        // PRESET 3 = MID CONE              (MID_CONE_INDEX)
        // PRESET 4 = HIGH CONE             (HIGH_CONE_INDEX)
        // PRESET 5 = FLOOR CUBE            (FLOOR_CUBE_INDEX)
        // PRESET 6 = CUBE HIGH             (HIGH_CUBE_INDEX)

        public static final int ZERO_POSITION_INDEX = 0;
        public static final int HUMAN_STATION_INDEX = 1;
        public static final int FLOOR_CONE_INDEX = 2;
        public static final int MID_CONE_INDEX = 3;
        public static final int HIGH_CONE_INDEX = 4;
        public static final int FLOOR_CUBE_INDEX = 5;
        public static final int HIGH_CUBE_INDEX = 6;

        public static final PresetList ROTATION_PRESETS = new PresetList(0.0, -47.987, -137.0, -57.0, -59.0, -110.0, -65.0);

        // pre-distance measurements
        //public static final PresetList EXTENSION_PRESETS = new PresetList(0.0, 30.261, 22.0, 17.0, 86.7, 10.619, 67.049);
        public static final PresetList EXTENSION_PRESETS = new PresetList(0.0, 17.365, 12.625, 9.755, 49.753, 6.09, 38.47);

        public static final PresetList WRIST_PRESETS = new PresetList(0.0, 33.0, -48.0, 10.0, 0.685, 58.0, -55.285);

        public static final PresetGroup CLIMBER_PRESET_GROUP = new PresetGroup()
                .addPreset(ROTATION_NAME, ROTATION_PRESETS)
                .addPreset(EXTENSION_NAME, EXTENSION_PRESETS)
                .addPreset(WRIST_NAME, WRIST_PRESETS);
    }


    public static class AutoValues {
        // fancy calculus type stuff, not sure what to do with it but play with the numbers ;)
        public static final PIDController X_CONTROLLER = new PIDController(0.1, 0, 0);
        public static final PIDController Y_CONTROLLER = new PIDController(0.1, 0, 0);

        public static final TunablePIDController PITCH_CONTROLLER = new TunablePIDController("Charge Pitch", 0.0081, 0.0, 0.0);

        public static final Map<String, Command> AUTO_EVENT_MAP = new HashMap<>();

        public static final String CHARGE_STATION_AUTO = "CHARGE STATION AUTO";
    }

    /**
     * This {@link Chassis} class is designed to store values regarding Driving the robot. This includes any offsets
     * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
     * need to be flipped due to the Field Oriented driving system.
     */
    public static class Chassis {
        /** The offset of the Front Right Motor */
        //public static final double FR_OFFSET = 0;

        public static final double FR_OFFSET = ((-2.38)+0)+(Math.PI/2) - (2 * Math.PI) + (Math.PI);

        /** The offset of the Front Left Motor */
        public static final double FL_OFFSET = ((9.401)+0.045647)+(Math.PI/2) - (Math.PI / 2);

        /** The offset of the Back Right Motor */
        public static final double BR_OFFSET =  ((-3.345)+0.009)+(Math.PI/2) - (Math.PI / 2) - (2 * Math.PI);



        /** The offset of the Back Left Motor */
        public static final double BL_OFFSET = ((((6.12)+0.339057)+(Math.PI/2) - (2 * Math.PI) - (Math.PI / 2)) * Math.PI)-0.33;


        /** The dead-zone where anything below this value, nothing will happen. */
        public static final double DRIVE_DEAD_ZONE = 0.15;

        /** The length of the side of the {@link Chassis} in <b>meters.</b> */
        public static final double CHASSIS_SIDE_LENGTH = 0.762;

        /** The Motor ID used for the Front Right Drive Motor. */
        public static final int FR_DRIVE_ID = 4;

        /** The Motor ID used for the Front Left Drive Motor. */
        public static final int FL_DRIVE_ID = 2;

        /** The Motor ID used for the Back Right Drive Motor. */
        public static final int BR_DRIVE_ID = 8;

        
        /** The Motor ID used for the Back Left Drive Motor. */
        public static final int BL_DRIVE_ID = 6;

        /** The Motor ID used for the Front Right Steering Motor. */
        public static final int FR_TURN_ID = 3;

        /** The Motor ID used for the Front Left Steering Motor. */
        public static final int FL_TURN_ID = 1;

        /** The Motor ID used for the Back Right Steering Motor. */
        public static final int BR_TURN_ID = 7;

        /** The Motor ID used for the Back Left Steering Motor. */
        public static final int BL_TURN_ID = 5;

        /** The ID used for the Front Right Absolute Encoder. */
        public static final int FR_DIO_ENCODER_PORT = 1;

        /** The ID used for the Front Left Absolute Encoder. */
        public static final int FL_DIO_ENCODER_PORT = 0;

        /** The ID used for the Back Right Absolute Encoder. */
        public static final int BR_DIO_ENCODER_PORT = 3;

        /** The ID used for the Back Left Absolute Encoder. */
        public static final int BL_DIO_ENCODER_PORT = 2;

        /** The Radius of each of the Swerve Drive Wheels in <b>meters.</b> */
        public static final double SWERVE_WHEEL_RADIUS = 0.0508;

        /** The Circumference of each of the Swerve Drive Wheels in <b>meters.</b> */
        public static final double SWERVE_WHEEL_CIRCUMFERENCE = SWERVE_WHEEL_RADIUS * 2 * Math.PI;

        /** How often the Odometry tracking of the Swerve Drive System is updated in <b>milliseconds.</b> */
        public static final double ODOMETRY_MS_INTERVAL = 5;

        /** The Error Factor for the Back Left. */
        public static final double BL_ERROR_FACTOR = 1;

        /** The Error Factor for the Back Right. */
        public static final double BR_ERROR_FACTOR = 1;

        /** The Error Factor for the Front Right. */
        public static final double FR_ERROR_FACTOR = 1;

        /** The Error Factor for the Front Left. */
        public static final double FL_ERROR_FACTOR = 1;

        public static final SwerveModule FL_MODULE = new SwerveModule(
                FL_DRIVE_ID,
                FL_TURN_ID,
                FL_DIO_ENCODER_PORT,
                FL_OFFSET,
                FL_ERROR_FACTOR
        ).setClosedLoopSupplier(() -> SwerveDriveSubsystem.closedLoop);
        public static final SwerveModule FR_MODULE = new SwerveModule(
                FR_DRIVE_ID,
                FR_TURN_ID,
                FR_DIO_ENCODER_PORT,
                FR_OFFSET,
                FR_ERROR_FACTOR
        ).setClosedLoopSupplier(() -> SwerveDriveSubsystem.closedLoop);
        public static final SwerveModule BL_MODULE = new SwerveModule(
                BL_DRIVE_ID,
                BL_TURN_ID,
                BL_DIO_ENCODER_PORT,
                BL_OFFSET,
                BL_ERROR_FACTOR
        ).setClosedLoopSupplier(() -> SwerveDriveSubsystem.closedLoop);
        public static final SwerveModule BR_MODULE = new SwerveModule(
                BR_DRIVE_ID,
                BR_TURN_ID,
                BR_DIO_ENCODER_PORT,
                BR_OFFSET,
                BR_ERROR_FACTOR
        ).setClosedLoopSupplier(() -> SwerveDriveSubsystem.closedLoop);
    }

    public static class Control {
        public static final int XBOX_CONTROLLER_ID = 2;
        public static final int LEFT_STICK_ID = 0;
        public static final int RIGHT_STICK_ID = 1;
    }

}
