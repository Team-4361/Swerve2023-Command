package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ClimberPresets;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.SparkMaxAngledPIDSubsystem;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static frc.robot.Constants.FourBarWristValues.WRIST_GEAR_RATIO;
import static frc.robot.Constants.FourBarWristValues.WRIST_MOTOR_ID;

public class ClimberWristSubsystem extends SparkMaxAngledPIDSubsystem {

    public ClimberWristSubsystem() {
        super(
                "ClimberWrist",
                new GearRatio(WRIST_GEAR_RATIO),
                new CANSparkMax(WRIST_MOTOR_ID, kBrushed),
                kBrushed,
                0.01,
                0,
                0
        );
        setTolerance(0.2);
        setPresets(ClimberPresets.WRIST_PRESETS);
    }
}
