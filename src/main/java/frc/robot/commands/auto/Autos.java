package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.assist.PIDRotateCommand;
import frc.robot.subsystems.vacuum.VacuumSubsystem;

import static frc.robot.Constants.AutoValues.*;
import static frc.robot.Constants.ClimberPresets.CLIMBER_PRESET_GROUP;

public class Autos {

    public static Command autoConePushCommand() {
        return Commands.runOnce(() -> {
            Robot.swerveDrive.resetGyroCommand();
            Robot.swerveDrive.resetPosition();
            CLIMBER_PRESET_GROUP.setCurrentPreset(0);
        }).andThen(new ParallelRaceGroup(
                Commands.run(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(1)),
                new WaitCommand(3)
        ).andThen(new ParallelRaceGroup(
                Commands.run(() -> Robot.swerveDrive.stop()),
                new WaitCommand(2)
        )));
    }

    public static Command autoConeMiddleCommand() {
        return Commands.runOnce(() -> {
            Robot.swerveDrive.resetGyroCommand();
            Robot.swerveDrive.resetPosition();
            Robot.pump.activate();
            CLIMBER_PRESET_GROUP.setCurrentPreset(0);
        }).andThen(new ParallelRaceGroup(
                Commands.run(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(3)),
                new WaitCommand(3)
        ).andThen(new ParallelRaceGroup(
                Commands.run(() -> {
                    Robot.swerveDrive.stop();
                })),
                new WaitCommand(2)
        ).andThen(new SequentialCommandGroup(
            Commands.runOnce(() -> Robot.pump.deactivate())),
            Robot.pump.openVacuumCommand()
        ));
    }

    ///////////////////////////////////////////

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

    public static Command chargeStationAutoCommand() {
        return autoConePushCommand().andThen(
                new ParallelRaceGroup(
                        new PIDRotateCommand(180),
                        new WaitCommand(3)
                ).andThen(new ParallelRaceGroup(
                        new PIDGoToCommand(new Pose2d(new Translation2d(-18, 0), new Rotation2d(0))),
                        new WaitCommand(3)
                )).andThen(Commands.runOnce(() -> {
                    Robot.swerveDrive.resetGyroCommand();
                    Robot.swerveDrive.resetPosition();
                })
        ));
    }

    public static Command coneMiddleAutoCommand() {
        return autoConeMiddleCommand().andThen(
            new ParallelRaceGroup(
                new PIDGoToCommand(new Pose2d(new Translation2d(18, 0), new Rotation2d(0))),
                new WaitCommand(3)
            ).andThen(Commands.runOnce(() -> {
                Robot.swerveDrive.resetGyroCommand();
                Robot.swerveDrive.resetPosition();
                Robot.pump.deactivate();
            })
        ));
    }
}
