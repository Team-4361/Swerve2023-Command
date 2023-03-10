package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.assist.PIDRotateCommand;

import static frc.robot.Constants.ClimberPresets.CLIMBER_PRESET_GROUP;

public class Autos {
    public static Command simpleAutoCommand() {
        return Commands.runOnce(() -> {
            Robot.swerveDrive.resetGyroCommand();
            Robot.swerveDrive.resetPosition();
            CLIMBER_PRESET_GROUP.setCurrentPreset(0);
        }).andThen(new ParallelRaceGroup(
                Commands.run(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(1)),
                new WaitCommand(3)
        ).andThen(new ParallelRaceGroup(
                Robot.swerveDrive.run(() -> Robot.swerveDrive.autoDrive(-0.5, 0, 0)),
                new WaitCommand(3)
        )).andThen(Robot.swerveDrive.runOnce(() -> Robot.swerveDrive.stop())));
    }

    public static Command complexAutoCommand() {
        return simpleAutoCommand().andThen(new PIDRotateCommand(180)).andThen(Commands.runOnce(() -> {
            Robot.swerveDrive.resetGyroCommand();
            Robot.swerveDrive.resetPosition();
            Robot.swerveDrive.stop();
        }));
    }
}
