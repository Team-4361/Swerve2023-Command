package frc.robot.util.math;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.ArrayList;
import java.util.Collections;

public class CIRMotorGroup {
    private ArrayList<CANSparkMax> motorList;
    private long[] startTimes;
    private double motorSpeed;

    public CIRMotorGroup(CANSparkMax... motors) {
        motorList = new ArrayList<>();
        Collections.addAll(motorList, motors);
    }

    public CIRMotorGroup addMotor(CANSparkMax motor) {
        motorList.add(motor);
        return this;
    }

    public double get() {
        return motorSpeed;
    }

    public void set(double speed) {
        // Calculate the start times for the Motor.
        startTimes = new long[motorList.size()];
        motorSpeed = speed;

        if (speed == 0) {
            // Stop all motors instantly.
            motorList.forEach(CANSparkMax::stopMotor);
        } else {
            long time = System.currentTimeMillis();
            for (int i = 0; i < motorList.size(); i++) {
                startTimes[i] = time;
                time += 250;
            }
        }
    }

    public void execute() {
        if (motorSpeed != 0 && startTimes.length > 0) {
            int index = 0;
            for (long time: startTimes) {
                if (System.currentTimeMillis() >= time) {
                    motorList.get(index).set(motorSpeed);
                }
                index++;
            }
        }
    }
}
