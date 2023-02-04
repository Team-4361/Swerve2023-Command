package frc.robot.subsystems.fourbar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMax.ControlType.kPosition;
import static com.revrobotics.SparkMaxPIDController.ArbFFUnits.kVoltage;
import static frc.robot.Constants.FourBarArmValues.*;

public class FourBarArmSubsystem extends SubsystemBase {

    private final CANSparkMax armMotor = new CANSparkMax(ARM_MOTOR_ID, ARM_MOTOR_TYPE);

    private final SparkMaxPIDController armController = armMotor.getPIDController();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final ArmFeedforward armFeedForward = new ArmFeedforward(ARM_kS, ARM_kG, ARM_kV, ARM_kA);

    private Rotation2d armAngle, targetAngle;
    private boolean teleopMode = false;

    ///////////////////////////////////////////////

    public Rotation2d getArmAngle() { return armAngle; }
    public Rotation2d getTargetAngle() { return targetAngle; }

    private double degreesToMotorRotations(double degrees) { return (ARM_GEAR_RATIO / 360) * degrees; }

    public void setTargetDeg(double targetDegrees) { targetAngle = new Rotation2d(Units.degreesToRadians(targetDegrees)); }

    public void setTargetRad(double targetRadians) { this.targetAngle = new Rotation2d(targetRadians); }

    public void setTarget(Rotation2d target) { this.targetAngle = target; }

    public FourBarArmSubsystem() {
        this.targetAngle = new Rotation2d(0);

        armMotor.enableVoltageCompensation(12);

        armController.setP(ARM_kP);
        armController.setI(ARM_kI);
        armController.setD(ARM_kD);
    }

    public void translateArm(double speed) {
        if (speed == 0 && teleopMode) {
            // Set the target angle to the current rotations to freeze the value and prevent the PIDController from
            // automatically adjusting to the previous value.
            setTargetDeg(getArmAngle().getDegrees());
            teleopMode = false;
        }
        if (speed != 0 && !teleopMode)
            teleopMode = true;

        armMotor.set(speed);
    }

    @Override
    public void periodic() {
        armAngle = new Rotation2d(Units.degreesToRadians(((360 / ARM_GEAR_RATIO) * armEncoder.getPosition())));

        if (!teleopMode)
            armController.setReference(degreesToMotorRotations(targetAngle.getDegrees()), kPosition, 0,
                    armFeedForward.calculate(
                            armAngle.getRadians(),
                            Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity())), kVoltage);

        SmartDashboard.putString("FourBar Angle:", armAngle.toString());
        SmartDashboard.putString("FourBar Target Angle:", targetAngle.toString());
    }
}
