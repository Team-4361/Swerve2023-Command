package frc.robot.subsystems.vacuum;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.VacuumValues.*;

public class VacuumSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private DoubleSolenoid solenoidR;


    public VacuumSubsystem() {
        this.motor = new CANSparkMax(VACUUM_MOTOR_ID, VACUUM_MOTOR_TYPE);
        solenoidR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOID_OPEN, SOLENOID_CLOSED);
        solenoidR.set(DoubleSolenoid.Value.kOff);
    
    }

    public void setSolenoid(boolean value) {
        if (value) {
            solenoidR.set(DoubleSolenoid.Value.kForward);
        } else {
            solenoidR.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void setSolenoid(DoubleSolenoid.Value state) {
        solenoidR.set(state);
    }

    public void activate(double speed) {
        motor.set(speed);
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
    }


}


