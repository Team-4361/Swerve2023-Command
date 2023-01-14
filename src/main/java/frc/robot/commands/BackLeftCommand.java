package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class BackLeftCommand extends CommandBase {
    @Override
    public void initialize() {
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void execute() {
        Robot.swerveDrive.autoDrive(-0.3,-0.5,0);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
