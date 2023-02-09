package frc.robot.util.pid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkMax.ControlType.kPosition;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class SparkMaxPIDSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final SparkMaxPIDController controller;
    private final RelativeEncoder encoder;
    private final String name;

    private double targetRotation;
    private boolean teleopMode;

    public void setTarget(double rotation) { this.targetRotation = rotation; }
    public double getRotation() { return encoder.getPosition(); }
    public double getTargetRotation() { return targetRotation; }

    public void translateMotor(double speed) {
        if (speed == 0 && teleopMode) {
            // Set the target angle to the current rotations to freeze the value and prevent the PIDController from
            // automatically adjusting to the previous value.
            setTarget(getRotation());
            teleopMode = false;
        }
        if (speed != 0 && !teleopMode)
            teleopMode = true;

        motor.set(speed);
    }

    /**
     * Checks if the actual value is within a specified tolerance of the expected value
     * @param expected The value to be expected.
     * @param actual The actual value.
     * @param tolerance The maximum error or tolerance that the value can be offset to still be true.
     * @return True/false depending on tolerance.
     */
    private static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    public boolean atTarget(double tolerance) {
        return inTolerance(getRotation(), getTargetRotation(), tolerance);
    }

    public SparkMaxPIDSubsystem(String name, CANSparkMax motor, double kP, double kI, double kD) {
        assert motor.getMotorType() != kBrushed : "PID Motor Cannot Be Brushed!";

        this.motor = motor;
        this.name = name;
        this.controller = motor.getPIDController();
        this.encoder = motor.getEncoder();
        this.targetRotation = 0;
        this.teleopMode = false;

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);

        motor.enableVoltageCompensation(12);
    }

    public SparkMaxPIDSubsystem(String name, int motorID, double kP, double kI, double kD) {
        this(name, new CANSparkMax(motorID, kBrushless), kP, kI, kD);
    }

    public SparkMaxPIDSubsystem(String name, int motorID) {
        this(name, new CANSparkMax(motorID, kBrushless), 0.01, 0, 0);
    }

    @Override
    public void periodic() {
        if (!teleopMode)
            controller.setReference(getTargetRotation(), kPosition);

        SmartDashboard.putNumber(name + " Rotation:", getRotation());
        SmartDashboard.putNumber(name + " Target Rotation:", getTargetRotation());
    }
}
