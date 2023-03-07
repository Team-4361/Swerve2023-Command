package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class SimpleAutoCommand extends SequentialCommandGroup {
    public SimpleAutoCommand() {
        super(
                Robot.swerveDrive.run(() -> Robot.swerveDrive.driveBack()),
                new WaitCommand(3),
                Robot.swerveDrive.runOnce(() -> Robot.swerveDrive.stop())
        );
    }
}
