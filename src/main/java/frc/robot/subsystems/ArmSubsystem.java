package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Arm.ARM_FEED_FWD;
import static frc.robot.Constants.Arm.ARM_MOTOR_ID;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor;
    private SparkMaxPIDController armController;

    private double targetPosition = 0;

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    @Override
    public void periodic() {
        armController.setReference(targetPosition, ControlType.kPosition);
    }

    public ArmSubsystem() {
        armMotor = new CANSparkMax(ARM_MOTOR_ID, kBrushless);
        armController = armMotor.getPIDController();

        armController.setFF(ARM_FEED_FWD);
        armMotor.enableVoltageCompensation(12);
    }
}
