package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.util.pid.VariablePose2d;

import static frc.robot.Constants.ClimberPresets.*;

public class Autos {
    public static class Feature {
        public static Command initAutoFeature() {
            return Commands.runOnce(() -> {
                Robot.swerveDrive.resetGyroCommand();
                Robot.swerveDrive.resetPosition();
                CLIMBER_PRESET_GROUP.setPreset(ZERO_POSITION_INDEX);
            });
        }

        public static Command middleConeDropFeature() {
            return new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        Robot.pump.activate();
                        CLIMBER_PRESET_GROUP.setPreset(MID_CONE_INDEX);
                    }),
                    new WaitCommand(3),
                    Commands.runOnce(() -> Robot.pump.deactivate()),
                    Robot.pump.openVacuumCommand()
            );
        }

        public static Command highCubeSyncDropFeature() {
            return new SequentialCommandGroup(
                    Robot.pump.activateCommand(),
                    CLIMBER_PRESET_GROUP.setPresetSyncCommand(HIGH_CONE_INDEX),
                    new WaitCommand(2),
                    Robot.pump.deactivateCommand(),
                    Robot.pump.openVacuumCommand()
            );
        }

        public static Command highCubeDropFeature() {
            return new SequentialCommandGroup(
                    Commands.runOnce(() -> {
                        Robot.pump.activate();
                        // NOTE: this is normal! high cone preset works with high cube
                        CLIMBER_PRESET_GROUP.setPreset(HIGH_CONE_INDEX);
                    }),
                    new WaitCommand(2),
                    Commands.runOnce(() -> Robot.pump.deactivate()),
                    Robot.pump.openVacuumCommand()
            );
        }
    }

    public static class AutoCommand {
        public static Command midConeNoStationCommand() {
            return new SequentialCommandGroup(
                    Feature.initAutoFeature(),
                    Feature.middleConeDropFeature(),
                    new TimeoutCommand(new PIDTranslateCommand(new VariablePose2d(-22, 0)), 5),
                    new ParallelRaceGroup(
                            Commands.run(() -> Robot.swerveDrive.stop()),
                            new WaitCommand(2)
                    )
            );
        }

        public static Command midConeAutoBalanceCommand() {
            return new SequentialCommandGroup(
                    Feature.initAutoFeature(),
                    Feature.middleConeDropFeature(),
                    CLIMBER_PRESET_GROUP.setPresetCommand(0),
                    new TimeoutCommand(new PIDTranslateCommand(new VariablePose2d(-22, 0)), 5),
                    new TimeoutCommand(Commands.runOnce(() -> Robot.swerveDrive.stop()), 0.5),
                    new TimeoutCommand(new PIDTranslateCommand(new VariablePose2d(-8.75, 0)), 3),
                    new ParallelRaceGroup(
                            new PIDAutoBalanceCommand(),
                            new WaitCommand(5)
                    ),
                    new PrintCommand("CHARGE STATION AUTO COMPLETE")
            );
        }

        public static Command highCubeAutoBalanceCommand() {
            return new SequentialCommandGroup(
                    Feature.initAutoFeature(),
                    Feature.highCubeDropFeature(),
                    CLIMBER_PRESET_GROUP.setPresetCommand(0),
                    new TimeoutCommand(new PIDTranslateCommand(new VariablePose2d(-22, 0)), 4),
                    Commands.runOnce(() -> Robot.swerveDrive.stop()),
                    new TimeoutCommand(new PIDTranslateCommand(new VariablePose2d(-8.75, 0)), 3),
                    new TimeoutCommand(new PIDAutoBalanceCommand(), 5),
                    new PrintCommand("CHARGE STATION AUTO COMPLETE")
            );
        }

        public static Command highDropOnlyCommand() {
            return new SequentialCommandGroup(
                Feature.initAutoFeature(),
                Feature.highCubeDropFeature(),
                new WaitCommand(4),
                CLIMBER_PRESET_GROUP.setPresetCommand(0)
            );
        }

        public static Command highCubeAdditionalCommand() {
            return new SequentialCommandGroup(
                    highCubeNoStationCommand(),
                    CLIMBER_PRESET_GROUP.setPresetCommand(FLOOR_CUBE_INDEX),
                    new GrabCubeCommand(),
                    CLIMBER_PRESET_GROUP.setPresetCommand(0)
            );
        }

        public static Command highCubeNoStationCommand() {
            return new SequentialCommandGroup(
                    Feature.initAutoFeature(),
                    Feature.highCubeDropFeature(),
                    CLIMBER_PRESET_GROUP.setPresetCommand(0),
                    new TimeoutCommand(new PIDTranslateCommand(new VariablePose2d(-22, 0)), 5),
                    new TimeoutCommand(Commands.run(() -> Robot.swerveDrive.stop()), 0.5),
                    new PrintCommand("NO CHARGE STATION AUTO COMPLETE")
            );
        }
    }
}