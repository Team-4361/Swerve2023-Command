// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PresetList;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

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

        public static final int TAG_PIPELINE = 0;
        public static final int CUBE_PIPELINE = 1;
        public static final int CONE_PIPELINE = 2;
        public static final int DRIVER_PIPELINE = 3;
    }

    public static class VacuumValues {
        public static int VACUUM_MOTOR_ID = 20;
        public static double VACUUM_PUMP_SPEED = 0.45;
        public static MotorType VACUUM_MOTOR_TYPE = kBrushed;
    }

    public static class FourBarArmValues {
        public static final int ARM_MOTOR_ID = 1;
        public static final MotorType ARM_MOTOR_TYPE = kBrushless;

        // The feed forward values for the arm. These can be automatically calculated by using ReCalc. Having
        // a correct feed forward is important as it compensates for the gravity and resistance that will push
        // the arm down when power is cut.
        public static final double ARM_kS = 0;
        public static final double ARM_kG = 0;
        public static final double ARM_kV = 0;
        public static final double ARM_kA = 0;

        public static final double ARM_kP = 0.01;
        public static final double ARM_kI = 0.00;
        public static final double ARM_kD = 0.00;

        public static final double ARM_GEAR_RATIO = 686; /*:1*/

        public static final double ARM_ANGLE_DEGREES = 10;
    }

    public static class FourBarWristValues {
        public static final int WRIST_GEAR_RATIO = 30;
        public static final int WRIST_MOTOR_ID = 5;

        public static final double[] WRIST_ANGLE_PRESETS = new double[]{
                5,
                10,
                15,
                20
        };
    }
    public static class FourBarGripperValues {
        public static final int GRIPPER_MOTOR_VALUE_ID = 0;
    }
    public static class ClimberArmValues {
        public static final int ROTATION_MOTOR_ID = 10;
        public static final int EXTENSION_MOTOR_ID = 18;
        public static final GearRatio ROTATION_GEAR_RATIO = GearRatio.fromRatio(1029);
    }

    public static class ClimberPresets {
        public static final PresetList<Double> ROTATION_PRESETS = new PresetList<>(1.0, 2.0, 3.0, 4.0);
        public static final PresetList<Double> EXTENSION_PRESETS = new PresetList<>(1.0, 2.0, 3.0, 4.0);
        public static final PresetList<Double> WRIST_PRESETS = new PresetList<>(1.0, 2.0, 3.0, 4.0);
    }


    public static class AutoValues {
        // fancy calculus type stuff, not sure what to do with it but play with the numbers ;)
        public static final Constraints X_CONSTRAINTS = new Constraints(0.2, 0.2);
        public static final Constraints Y_CONSTRAINTS = new Constraints(0.2,0.2);
        public static final Constraints OMEGA_CONSTRAINTS = new Constraints(0.2, 0.2);
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
        public static final double BL_OFFSET = ((6.12)+0.339057)+(Math.PI/2) - (2 * Math.PI) - (Math.PI / 2);


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
