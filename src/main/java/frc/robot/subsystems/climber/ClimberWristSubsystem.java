package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ClimberPresets;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.SparkMaxAngledPIDSubsystem;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static frc.robot.Constants.ClimberPresets.CLIMBER_PRESET_GROUP;
import static frc.robot.Constants.ClimberPresets.WRIST_NAME;
import static frc.robot.Constants.FourBarWristValues.WRIST_GEAR_RATIO;
import static frc.robot.Constants.FourBarWristValues.WRIST_MOTOR_ID;

public class ClimberWristSubsystem extends SparkMaxAngledPIDSubsystem {

    public ClimberWristSubsystem() {
        super(
                WRIST_NAME,
                new GearRatio(WRIST_GEAR_RATIO),
                new CANSparkMax(WRIST_MOTOR_ID, kBrushed),
                0.01,
                0,
                0
        );
        setTolerance(0.2);
        setPresetSupplier(() -> CLIMBER_PRESET_GROUP.getCurrentPreset(WRIST_NAME));
    }
}