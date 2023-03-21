package frc.robot.commands.assist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ClimberPresets.CLIMBER_PRESET_GROUP;

public class VerticalGrabCommand extends CommandBase {
    private long endTime = Long.MIN_VALUE;

    public VerticalGrabCommand() {
        addRequirements(Robot.arm, Robot.pump);
    }

    @Override
    public void initialize() {
        if (endTime == Long.MIN_VALUE) {
            endTime = System.currentTimeMillis() + 2000;
        }
        Robot.pump.activate(); // ensure the pump is running first!
    }

    @Override
    public void execute() {
        Robot.arm.getRotation().translateMotor(-0.3);
    }

    @Override
    public boolean isFinished() {
        return Robot.pump.hasPressure() || System.currentTimeMillis() >= endTime || Robot.arm.getExtension().getRotation() >= -110.0;
    }

    @Override
    public void end(boolean interrupted) {
        // Do not stop the pump since it will let go otherwise!
        Robot.arm.getRotation().translateMotor(0);
    }
}
