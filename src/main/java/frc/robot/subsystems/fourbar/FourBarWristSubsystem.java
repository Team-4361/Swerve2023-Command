package frc.robot.subsystems.fourbar;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FourBarWristValues.WRIST_SERVO_ID;

public class FourBarWristSubsystem extends SubsystemBase {
    private final Servo wristServo;
    public FourBarWristSubsystem(){
         wristServo = new Servo(WRIST_SERVO_ID);
    }

    public void setAngleDeg(double angle) {
        wristServo.setAngle(angle);
    }

    public double getAngleDeg() {
        return wristServo.getAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FourBarWrist: Angle", wristServo.getAngle());
    }
}
