package frc.robot.subsystems.vacuum;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import static frc.robot.Constants.VacuumValues.*;

public class VacuumSubsystem extends SubsystemBase {
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;

    private final MotorControllerGroup motor;
    private final Solenoid solenoidOne, solenoidTwo;
    private final PowerDistribution pdh;
    private final AnalogInput sensorOne, sensorTwo;

    private boolean ledStatus = false;


    public VacuumSubsystem() {
        this.motor = new MotorControllerGroup(
            new CANSparkMax(VACUUM_MOTOR_ID[0], VACUUM_MOTOR_TYPE),
            new CANSparkMax(VACUUM_MOTOR_ID[1], VACUUM_MOTOR_TYPE)
        );
        pdh = new PowerDistribution();
        solenoidOne = new Solenoid(MODULE_TYPE, VACUUM_SOLENOID_ONE);
        solenoidTwo = new Solenoid(MODULE_TYPE, VACUUM_SOLENOID_TWO);
        sensorOne = new AnalogInput(VACUUM_SENSOR_ONE);
        sensorTwo = new AnalogInput(VACUUM_SENSOR_TWO);
    }

    public Command openVacuumCommand() {
        return new ParallelRaceGroup(
                Commands.run(() -> {
                    solenoidOne.set(true);
                    solenoidTwo.set(true);
                }),
                new WaitCommand(2)
        ).andThen(Commands.runOnce(() -> {
            solenoidOne.set(false);
            solenoidTwo.set(false);
        }));
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

        SmartDashboard.putNumber("Vacuum: Sensor 1", sensorOne.getValue());
        SmartDashboard.putNumber("Vacuum: Sensor 2", sensorTwo.getValue());

        SmartDashboard.putBoolean("Vacuum: Bound", sensorOne.getValue()>=VACUUM_THRESHOLD || sensorTwo.getValue()>=VACUUM_THRESHOLD);

        pdh.setSwitchableChannel(ledStatus);

    }
}


