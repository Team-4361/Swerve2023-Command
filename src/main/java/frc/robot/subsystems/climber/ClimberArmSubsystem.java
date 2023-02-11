package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.pid.SparkMaxPIDSubsystem;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.ClimberArmValues.*;

public class ClimberArmSubsystem extends SubsystemBase {
    private final SparkMaxPIDSubsystem extension, rotation;
    private final Solenoid solenoid;
    private Servo servo;
    public ClimberArmSubsystem() {
        this.extension = new SparkMaxPIDSubsystem("Climber Extension", EXTENSION_MOTOR_ID);
        this.rotation = new SparkMaxPIDSubsystem("Climber Rotation", ROTATION_MOTOR_ID);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID_ID);
        servo = new Servo(SERVO_ID);
    }

    public SparkMaxPIDSubsystem getExtension() { return extension; }
    public SparkMaxPIDSubsystem getRotation() { return rotation; }

    public void openSolenoid(){
        solenoid.set(true);
    }
    public void closeSolenoid(){
        solenoid.set(false);
    }
    public void setAngle(){
        servo.setAngle(180);
    }

}
