package frc.robot.subsystems.fourbar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.SparkMaxPIDSubsystem;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static frc.robot.Constants.FourBarWristValues.*;

public class FourBarWristSubsystem extends SparkMaxPIDSubsystem {
    @Override
    public double getRotation() {
        return GearRatio.motorRotationsToDegrees(super.getRotation(), WRIST_GEAR_RATIO);
    }

    @Override
    public void setTarget(double degrees) {
        super.setTarget(degrees);
    }

    public FourBarWristSubsystem() {
        super(
                "FourBarWrist",
                new CANSparkMax(WRIST_MOTOR_ID, kBrushed),
                0.01,
                0,
                0
        );
        setTolerance(2);
    }
}
