package frc.robot.subsystems.fourbar;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static frc.robot.Constants.FourBarGripperValues.GRIPPER_MOTOR_VALUE_ID;

public class FourBarGripperSubsystem extends SubsystemBase {
    private final CANSparkMax gripperMotor;

    public FourBarGripperSubsystem() {
        gripperMotor = new CANSparkMax(GRIPPER_MOTOR_VALUE_ID, kBrushed);
    }

    public void openGripper(double speed) { gripperMotor.set(speed); }

    public void closeGripper(double speed) { gripperMotor.set(-speed); }

    public void stopGripper() { gripperMotor.stopMotor(); }
}
