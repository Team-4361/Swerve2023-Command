package frc.robot.subsystems.fourbar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.pid.SparkMaxPIDSubsystem;

import static com.revrobotics.CANSparkMax.ControlType.kPosition;
import static frc.robot.Constants.FourBarArmValues.*;

public class FourBarArmSubsystem extends SparkMaxPIDSubsystem {
    public FourBarArmSubsystem() {
        super(
                "FourBar",
                ARM_MOTOR_ID,
                ARM_kP,
                ARM_kI,
                ARM_kD
        );
    }
}
