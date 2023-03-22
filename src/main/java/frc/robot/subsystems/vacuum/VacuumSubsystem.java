package frc.robot.subsystems.vacuum;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import static frc.robot.Constants.TEST_MODE;
import static frc.robot.Constants.VacuumValues.*;

public class VacuumSubsystem extends SubsystemBase {
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;

    private final MotorControllerGroup motor;
    private final Solenoid solenoidOne, solenoidTwo, solenoidThree;
    private final PowerDistribution pdh;
    private final AnalogInput sensorOne, sensorTwo;

    private boolean ledStatus = false;
    private boolean vacEnabled = false;

    public void toggleVacuum() {
        if (motor.get() == 0) {
            activate();
        } else if (motor.get() != 0) {
            deactivate();
        }
    }

    public boolean hasPressure() {
        return sensorOne.getVoltage()<=VACUUM_THRESHOLD || sensorTwo.getVoltage()<=VACUUM_THRESHOLD;
    }

    public VacuumSubsystem() {
        this.motor = new MotorControllerGroup(
            new CANSparkMax(VACUUM_MOTOR_ID[0], VACUUM_MOTOR_TYPE),
            new CANSparkMax(VACUUM_MOTOR_ID[1], VACUUM_MOTOR_TYPE),
            new CANSparkMax(VACUUM_MOTOR_ID[2], VACUUM_MOTOR_TYPE),
            new CANSparkMax(VACUUM_MOTOR_ID[3], VACUUM_MOTOR_TYPE)
        );
        pdh = new PowerDistribution();
        solenoidOne = new Solenoid(MODULE_TYPE, VACUUM_SOLENOID_ONE);
        solenoidTwo = new Solenoid(MODULE_TYPE, VACUUM_SOLENOID_TWO);
        solenoidThree = new Solenoid(MODULE_TYPE, VACUUM_SOLENOID_THREE);

        sensorOne = new AnalogInput(VACUUM_SENSOR_HORIZONTAL);
        sensorTwo = new AnalogInput(VACUUM_SENSOR_VERTICAL);
    }

    public Command openVacuumCommand() {
        return new ParallelRaceGroup(
                Commands.run(() -> {
                    // disable the vacuum until the solenoid shuts off.
                    motor.set(0);
                    solenoidOne.set(true);
                    solenoidTwo.set(true);
                    solenoidThree.set(true);
                }),
                new WaitCommand(2)
        ).andThen(Commands.runOnce(() -> {
            solenoidOne.set(false);
            solenoidTwo.set(false);
            solenoidThree.set(false);
            if (vacEnabled) {
                motor.set(VACUUM_PUMP_SPEED);
            }
        }));
    }


    public Command toggleLEDCommand() {
        return this.runOnce(() -> ledStatus = !ledStatus);
    }

    public void activate() {
        motor.set(VACUUM_PUMP_SPEED);
        vacEnabled = true;
    }

    public void deactivate() {
        motor.set(0);
        vacEnabled = false;
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

        if (TEST_MODE) {
            SmartDashboard.putNumber("Vacuum: Power", motor.get());
        }

        SmartDashboard.putNumber("Vacuum: Sensor 1", sensorOne.getVoltage());
        SmartDashboard.putNumber("Vacuum: Sensor 2", sensorTwo.getVoltage());

        SmartDashboard.putBoolean("Vacuum: Bound", hasPressure());
        SmartDashboard.putBoolean("Vacuum: Solenoid", solenoidOne.get() || solenoidTwo.get() || solenoidThree.get());
        pdh.setSwitchableChannel(ledStatus);
    }
}


