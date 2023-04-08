package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import static frc.robot.Constants.ClimberPresets.*;

public class GrabCubeCommand extends CommandBase {
    @Override
    public void initialize() {
        CLIMBER_PRESET_GROUP.setPreset(GRAB_FLOOR_CUBE_NAME);
        Robot.pump.activate();
    }

    @Override
    public void end(boolean interrupted) {
        CLIMBER_PRESET_GROUP.setPreset(ZERO_POSITION_NAME);
    }

    @Override
    public boolean isFinished() {
        return Robot.pump.sensors.isAnyBound();
    }
}
