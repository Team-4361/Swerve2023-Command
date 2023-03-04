package frc.robot.commands.swerve.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class AutoVacuumCommand extends SequentialCommandGroup {
    public AutoVacuumCommand() {
        super(
                Robot.pump.runOnce(() -> Robot.pump.activate(0.45))
        );
    }
}
