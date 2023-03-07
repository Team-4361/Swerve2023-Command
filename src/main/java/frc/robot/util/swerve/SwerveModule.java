package frc.robot.util.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.Supplier;

import static frc.robot.Constants.Chassis.*;

/**
 * A {@link SwerveModule} is composed of two motors and two encoders:
 * a drive motor/encoder and a turn motor/encoder. The turn motor is
 * responsible for controlling the direction the drive motor faces, essentially
 * allowing the robot to move in any direction.
 */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder rotationPWMEncoder;

    private final double offset;
    private final double errorFactor;
    private final PIDController turnController = new PIDController(
            0.5,
            0.0,
            0.0,
            0.02
    );

    private final PIDController driveController = new PIDController(
            0.01,
            0.0,
            0
    );

    private Supplier<Boolean> closedLoopSupplier = () -> false;

    /**
     * Creates a new {@link SwerveModule} instance, using the specified parameters.
     *
     * @param driveMotorId       The Motor ID used for driving the wheel.
     * @param turnMotorId        The Motor ID used for turning the wheel.
     * @param digitalEncoderPort The {@link DigitalInput} ID used for the Encoder.
     * @param offset             The offset to use for driving the wheel.
     * @param errorFactor        The maximum error factor that is acceptable.
     */
    public SwerveModule(int driveMotorId, int turnMotorId, int digitalEncoderPort, double offset, double errorFactor) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationPWMEncoder = new DutyCycleEncoder(digitalEncoderPort);

        this.offset = offset;
        this.errorFactor = errorFactor;
    }

    public SwerveModule setClosedLoopSupplier(Supplier<Boolean> supplier) {
        this.closedLoopSupplier = supplier;
        return this;
    }

    /**
     * @return The current meters per second of the robot.
     */
    private double velocityMetersPerSecond() {
        // rpm -> rps -> mps
        double rotationsPerMinute = driveEncoder.getVelocity();
        double rotationsPerSecond = rotationsPerMinute / 60;

        return (rotationsPerSecond * SWERVE_WHEEL_CIRCUMFERENCE)/3.3;
    }

    public double getDriveMPH() {
        return (velocityMetersPerSecond() * 2.237);
    }

    public Rotation2d getTurnAngle() {
        return new Rotation2d(turnAngleRadians());
    }

    public double turnAngleRadians() {
        return offset + (rotationPWMEncoder.get() * 2 * Math.PI);
    }

    public double turnAngleRadiansNoOffset() {
        return (rotationPWMEncoder.get() * 2 * Math.PI);
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getTurnAngle());

        double turnPower = turnController.calculate(
                turnAngleRadians(),
                state.angle.getRadians()
        );

        double drivePower;
        if (closedLoopSupplier.get()) {
            drivePower = driveController.calculate(velocityMetersPerSecond()*CHASSIS_MAX_SPEED, state.speedMetersPerSecond*CHASSIS_MAX_SPEED);
        } else {
            drivePower = (state.speedMetersPerSecond / CHASSIS_MAX_SPEED) * errorFactor;
        }

        driveMotor.set(drivePower);
        turnMotor.set(turnPower);
    }

    /**
     * Get the {@link SwerveModuleState} based on the drive motor's velocity
     * (meters/sec) and the turn encoder's angle.
     *
     * @return a new {@link SwerveModuleState}, representing the module's
     * current state, based on the module's drive motor velocity (m/s)
     * and the turn encoder's angle.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                velocityMetersPerSecond(),
                getTurnAngle()
        );
    }

    /**
     * Get the {@link SwerveModulePosition} based on the drive motor's
     * distance travelled (in meters), and turn encoder's angle. This
     * is required for {@link SwerveOdometry} to work correctly.
     *
     * @return A {@link SwerveModulePosition}, representing the module's
     * current position, based on the module's drive motor distance and
     * the turn encoder's angle.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getMetersDriven(),
                getTurnAngle()
        );
    }

    public double getRPM() {
        return driveEncoder.getVelocity();
    }

    public void updateDashboard(String prefix) {
        String driveVelocity = prefix + ": rpm";
        String drivePower = prefix + ": pow";
        String turnPower = prefix + ": turn pow";
        String turnPosition = prefix + ": turn rad";

        SmartDashboard.putNumber(driveVelocity, getRPM());
        SmartDashboard.putNumber(turnPower, turnMotor.get());
        SmartDashboard.putNumber(turnPosition, turnAngleRadians());
        SmartDashboard.putNumber(drivePower, driveMotor.get());
        SmartDashboard.putNumber(prefix + " offset tuning rad:", turnAngleRadiansNoOffset());
        SmartDashboard.putNumber(prefix + " drive encoder: ", driveEncoder.getPosition());

    }

    /**
     * get the elapsed distance, in rotations.
     *
     * @return the elapsed distance, in rotations
     */
    public double getRotations() {
        return driveEncoder.getPosition();
    }

    /**
     * @return The total amount of meters the individual {@link SwerveModule} has travelled.
     */
    public double getMetersDriven() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return (driveEncoder.getPosition() * (2 * Math.PI) * SWERVE_WHEEL_RADIUS);
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }
}