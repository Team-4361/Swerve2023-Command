package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberPresets;
import frc.robot.util.pid.SparkMaxAngledPIDSubsystem;
import frc.robot.util.pid.SparkMaxPIDSubsystem;

import static frc.robot.Constants.ClimberArmValues.*;
import static frc.robot.Constants.ClimberPresets.CLIMBER_PRESET_GROUP;

public class ClimberArmSubsystem extends SubsystemBase {
    private final SparkMaxPIDSubsystem extension, rotation;
    public ClimberArmSubsystem() {
        this.extension = new SparkMaxPIDSubsystem("Climber Extension", EXTENSION_MOTOR_ID);
        this.rotation = new SparkMaxAngledPIDSubsystem("Climber Rotation", ROTATION_GEAR_RATIO, ROTATION_MOTOR_ID);
    
        extension.setPresetSupplier(() -> CLIMBER_PRESET_GROUP.getCurrentPreset("Climber Extension"));
        rotation.setPresetSupplier(() -> CLIMBER_PRESET_GROUP.getCurrentPreset("Climber Rotation"));
    }

    public SparkMaxPIDSubsystem getExtension() { return extension; }
    public SparkMaxPIDSubsystem getRotation() { return rotation; }
}
