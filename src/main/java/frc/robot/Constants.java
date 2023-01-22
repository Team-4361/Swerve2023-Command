// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * This {@link FrontCamera} class is designed to contain the required values for the PhotonVision
     * AprilTag tracking system.
     */
    public static class FrontCamera {
        public static final String CAMERA_NAME = "photoncamera";
        public static final double CAMERA_HEIGHT_METERS = 0.60;
        public static final double TARGET_HEIGHT_METERS = 0.60;
        public static final double CAMERA_PITCH = 0;

        public static final int APRIL_TAG_PIPELINE = 0;
        public static final int CUBE_PIPELINE = 1;
        public static final int CONE_PIPELINE = 2;
        public static final int DRIVER_PIPELINE = 3;
    }

    public static class AutoValues {
        // fancy calculus type stuff, not sure what to do with it but play with the numbers ;)
        public static final double PID_PROPORTIONAL = 0.5;
        public static final double PID_INTEGRAL = 0;
        public static final double PID_DERIVATIVE = 0;

        public static final double MAXIMUM_DRIVE_VELOCITY = 2; // meters per second
        public static final double MAXIMUM_TURN_VELOCITY = 2;
        public static final double MAXIMUM_TURN_ACC = Math.PI/2;
    }

    /**
     * This {@link Chassis} class is designed to store values regarding Driving the robot. This includes any offsets
     * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
     * need to be flipped due to the Field Oriented driving system.
     */
    public static class Chassis {
        /** The offset of the Front Right Motor */
        //public static final double FR_OFFSET = 0;

        public static final double FR_OFFSET = -2.38 - (2 * Math.PI) + (Math.PI);
        /** The offset of the Front Left Motor */
        public static final double FL_OFFSET = -9.401 - (Math.PI / 2);

        /** The offset of the Back Right Motor */
        public static final double BR_OFFSET =  -3.345 - (Math.PI / 2) - (2 * Math.PI);



        /** The offset of the Back Left Motor */
        public static final double BL_OFFSET = -6.12 - (2 * Math.PI) - (Math.PI / 2);


        /** The dead-zone where anything below this value, nothing will happen. */
        public static final double DRIVE_DEAD_ZONE = 0.15;

        /** The length of the side of the {@link Chassis} in <b>meters.</b> */
        public static final double SWERVE_CHASSIS_SIDE_LENGTH = 0.762;

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
    }

    public static class Control {
        public static final int XBOX_CONTROLLER_ID = 2;
        public static final int LEFT_STICK_ID = 0;
        public static final int RIGHT_STICK_ID = 1;
    }
}
