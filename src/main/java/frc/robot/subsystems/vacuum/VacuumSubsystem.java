package frc.robot.subsystems.vacuum;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import static frc.robot.Constants.VacuumValues.*;

public class VacuumSubsystem extends SubsystemBase {
    private final Solenoid vacuumSolenoid;
    private final MotorControllerGroup motor;

    private final Solenoid ledSolenoid;
    private final Solenoid ledSolenoid2;

    private boolean ledStatus = false, updating = true;

    private PowerDistribution pdh;

    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;
    

    public VacuumSubsystem() {
        this.motor = new MotorControllerGroup(
            new CANSparkMax(VACUUM_MOTOR_ID[0], VACUUM_MOTOR_TYPE),
            new CANSparkMax(VACUUM_MOTOR_ID[1], VACUUM_MOTOR_TYPE)
        );
        pdh = new PowerDistribution();
        vacuumSolenoid = new Solenoid(MODULE_TYPE, VACUUM_SOLENOID_ID);
        ledSolenoid = new Solenoid(MODULE_TYPE, 7);
        ledSolenoid2 = new Solenoid(MODULE_TYPE, 3);
    }

    public Command openVacuumCommand() {
        return this.runOnce(() -> {
            updating = false;
            ledSolenoid.set(false);
            ledSolenoid2.set(false);
        }).andThen(new ParallelRaceGroup(
                this.run(() -> vacuumSolenoid.set(true)),
                new WaitCommand(2)
        ).andThen(this.runOnce(() -> vacuumSolenoid.set(false)))).andThen(
            this.runOnce(() -> {
                ledSolenoid.set(ledStatus);
                ledSolenoid2.set(ledStatus);
                updating = true;
            })
        );
    }


    public Command toggleLEDCommand() {
        return this.runOnce(() -> ledStatus = !ledStatus);
    }

    public void activate() {
        motor.set(VACUUM_PUMP_SPEED);
    }

    public void deactivate() {
        motor.set(0);
    }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vacuum: Running", motor.get() != 0);
        SmartDashboard.putNumber("Vacuum: Power", motor.get());

        if (updating) {
            ledSolenoid.set(ledStatus);
            ledSolenoid2.set(ledStatus);
            pdh.setSwitchableChannel(ledStatus);
        }
    }
}


