package frc.robot.subsystems.vacuum;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

import java.util.logging.Logger;

import static frc.robot.Constants.VacuumValues.*;

public class VacuumSubsystem extends SubsystemBase {
    private Solenoid solenoid;
    private MotorControllerGroup pumps;

    public VacuumSubsystem() {
        this.pumps = new MotorControllerGroup(
            new CANSparkMax(VACUUM_MOTOR_ID[0], VACUUM_MOTOR_TYPE),
            new CANSparkMax(VACUUM_MOTOR_ID[1], VACUUM_MOTOR_TYPE)
        );
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, VACUUM_SOLENOID_ID);
        //servo = new Servo(SERVO_ROTATION_ID);
        //servo.setBounds(2.43, 2.43, 1.49, 0.557, 0.557);
    }

    public void openVacuum() {
        solenoid.set(true);
    }

    public void closeVacuum() {
        solenoid.set(false);
    }

    public void setSolenoid(boolean value) {
        solenoid.set(value);
    }

    public Command openVacuumCommand() {
        return new SequentialCommandGroup(
                runOnce(this::openVacuum),
                new WaitCommand(2),
                runOnce(this::closeVacuum)
        );
    }

    public void activate(double speed) {
        pumps.set(speed);
    }

    public void deactivate() {
        pumps.set(0);
    }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vacuum: Running", pumps.get() != 0);
        SmartDashboard.putNumber("Vacuum: Power", pumps.get());
    }


}


