// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PresetGroup;
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
        public static int[] VACUUM_MOTORS = new int[]{20, 13};
        public static double VACUUM_PUMP_SPEED = 0.45;
        public static MotorType VACUUM_MOTOR_TYPE = kBrushed;
        public static int SOLENOID_OPEN = 0;
        public static int SOLENOID_CLOSED = 1;
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
        public static final int WRIST_MOTOR_ID = 22; // TODO: change!

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
        public static final int EXTENSION_MOTOR_ID = 21;
        public static final GearRatio ROTATION_GEAR_RATIO = GearRatio.fromRatio(1029);
    }

    public static class ClimberPresets {
        public static final String ROTATION_NAME = "Climber Rotation";
        public static final String EXTENSION_NAME = "Climber Extension";
        public static final String WRIST_NAME = "Climber Wrist";

        public static final PresetList ROTATION_PRESETS = new PresetList(1.0, 2.0, 3.0, 4.0);
        public static final PresetList EXTENSION_PRESETS = new PresetList(1.0, 2.0, 3.0, 4.0);
        public static final PresetList WRIST_PRESETS = new PresetList(1.0, 2.0, 3.0, 4.0);

        public static final PresetGroup CLIMBER_PRESET_GROUP = new PresetGroup()
                .addPreset(ROTATION_NAME, ROTATION_PRESETS)
                .addPreset(EXTENSION_NAME, EXTENSION_PRESETS)
                .addPreset(WRIST_NAME, WRIST_PRESETS);
    }

    public static class Control {
        public static final int XBOX_CONTROLLER_ID = 2;
        public static final int LEFT_STICK_ID = 0;
        public static final int RIGHT_STICK_ID = 1;
    }
}
