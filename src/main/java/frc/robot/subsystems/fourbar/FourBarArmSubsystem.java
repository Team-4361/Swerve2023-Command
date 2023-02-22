package frc.robot.subsystems.fourbar;

import frc.robot.Robot;
import frc.robot.util.pid.SparkMaxPIDSubsystem;

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
        setPIDControlSupplier(() -> Robot.pidControlEnabled);
    }
}
