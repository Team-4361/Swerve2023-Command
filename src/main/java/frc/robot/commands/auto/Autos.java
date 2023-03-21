package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

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

    public static Command autoConeMiddleFeature() {
        /*
        return Commands.runOnce(() -> {
            Robot.swerveDrive.resetGyroCommand();
            Robot.swerveDrive.resetPosition();
            Robot.pump.activate();
            CLIMBER_PRESET_GROUP.setCurrentPreset(0);
        }).andThen(new ParallelRaceGroup(Commands.run(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(3)), new WaitCommand(3))
        ).andThen(new SequentialCommandGroup(
            Commands.runOnce(() -> Robot.pump.deactivate())),
            Robot.pump.openVacuumCommand()
        );

         */
        return new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    Robot.swerveDrive.resetGyroCommand();
                    Robot.swerveDrive.resetPosition();
                    Robot.pump.activate();
                    CLIMBER_PRESET_GROUP.setCurrentPreset(3);
                }),
                new WaitCommand(3),
                Commands.runOnce(() -> Robot.pump.deactivate()),
                Robot.pump.openVacuumCommand()
        );
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


    public static Command coneMiddleChargeStationCommand() {
        /*
        return autoConeMiddleCommand().andThen(
            new ParallelRaceGroup(
                new PIDGoToCommand(new Pose2d(new Translation2d(-18, 0), new Rotation2d(0))),
                new WaitCommand(3)
            ).andThen(Commands.runOnce(() -> {
                Robot.swerveDrive.resetGyroCommand();
                Robot.swerveDrive.resetPosition();
                Robot.pump.deactivate();
            })
        ));

         */
        return new SequentialCommandGroup(
                autoConeMiddleFeature(),
                Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(0)),
                new ParallelRaceGroup(
                        new PIDGoToCommand(new Pose2d(new Translation2d(-20, 0), new Rotation2d(0))),
                        new WaitCommand(3)
                ),
                new ParallelRaceGroup(
                        new PIDGoToCommand(new Pose2d(new Translation2d(-8.75, 0), new Rotation2d(0))),
                        new WaitCommand(3)
                ),
                new ParallelRaceGroup(
                        new PIDAutoBalanceCommand(),
                        new WaitCommand(5)
                )
        );
    }
}
