package frc.robot.util.pid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.util.math.GearRatio;

public class SparkMaxAngledPIDSubsystem extends SparkMaxPIDSubsystem {
    private final GearRatio gearRatio;

    @Override
    public double getRotation() {
        return gearRatio.motorRotationsToDegrees(super.getRotation());
    }

    @Override
    public double getTargetRotation() {
        return gearRatio.motorRotationsToDegrees(super.getTargetRotation());
    }

    @Override
    public SparkMaxAngledPIDSubsystem setTolerance(double angle) {
        super.setTolerance(gearRatio.degreesToMotorRotations(angle));
        return this;
    }

    @Override
    public void setTarget(double angle) {
        super.setTarget(gearRatio.degreesToMotorRotations(angle));
    }

    public SparkMaxAngledPIDSubsystem(String name, GearRatio ratio, CANSparkMax motor, CANSparkMaxLowLevel.MotorType motorType, double kP, double kI, double kD) {
        super(name, motor, motorType, kP, kI, kD);
        this.gearRatio = ratio;
    }

    public SparkMaxAngledPIDSubsystem(String name, GearRatio ratio, int motorID, double kP, double kI, double kD) {
        super(name, motorID, kP, kI, kD);
        this.gearRatio = ratio;
    }

    public SparkMaxAngledPIDSubsystem(String name, GearRatio ratio, int motorID) {
        super(name, motorID);
        this.gearRatio = ratio;
    }


}
