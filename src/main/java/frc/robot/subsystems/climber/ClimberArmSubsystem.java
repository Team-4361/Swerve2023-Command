package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.pid.SparkMaxAngledPIDSubsystem;
import frc.robot.util.pid.SparkMaxPIDSubsystem;

import static frc.robot.Constants.ClimberArmValues.*;
import static frc.robot.Constants.ClimberPresets.*;

public class ClimberArmSubsystem extends SubsystemBase {
    private final SparkMaxPIDSubsystem extension;
    private final SparkMaxAngledPIDSubsystem rotation;

    public ClimberArmSubsystem() {
        this.extension = new SparkMaxPIDSubsystem(EXTENSION_NAME, EXTENSION_MOTOR_ID);
        this.rotation = new SparkMaxAngledPIDSubsystem(ROTATION_NAME, ROTATION_GEAR_RATIO, ROTATION_MOTOR_ID);
    
        extension.setPresetSupplier(() -> CLIMBER_PRESET_GROUP.getCurrentPreset(EXTENSION_NAME));
        rotation.setPresetSupplier(() -> CLIMBER_PRESET_GROUP.getCurrentPreset(ROTATION_NAME));

        extension.setPIDControlSupplier(() -> Robot.pidControlEnabled);
        rotation.setPIDControlSupplier(() -> Robot.pidControlEnabled);
    }

    public SparkMaxPIDSubsystem getExtension() { return extension; }
    public SparkMaxAngledPIDSubsystem getRotation() { return rotation; }
}
