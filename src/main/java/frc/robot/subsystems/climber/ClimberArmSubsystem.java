package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberArmValues.EXTENSION_MOTOR_ID;
import static frc.robot.Constants.ClimberArmValues.ROTATION_MOTOR_ID;

public class ClimberArmSubsystem extends SubsystemBase {
    private CANSparkMax rotationMotor;
    private CANSparkMax extensionMotor;
    private SparkMaxPIDController controller;
    private RelativeEncoder rotationEncoder;
    private double targetRotations;

    public ClimberArmSubsystem() {
        rotationMotor = new CANSparkMax(ROTATION_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        extensionMotor = new CANSparkMax(EXTENSION_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
        controller = rotationMotor.getPIDController();
        rotationEncoder = rotationMotor.getEncoder();
        targetRotations = 0;
    }
    public void setTargetRotation(double rotation){
        controller.setReference(rotation, CANSparkMax.ControlType.kPosition);
        targetRotations = rotation;
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber arm target", targetRotations);
        SmartDashboard.putNumber("Climber current rotations", rotationEncoder.getPosition());
    }
}
