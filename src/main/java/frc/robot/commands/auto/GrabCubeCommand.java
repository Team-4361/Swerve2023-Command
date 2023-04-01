package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class GrabCubeCommand extends SequentialCommandGroup {
    public GrabCubeCommand() {
        super(
                new TimeoutCommand(
                        Robot.arm.run(
                                () -> Robot.arm.getRotation().translateMotor(-0.25)
                        ).until(() -> Robot.pump.sensors.isAnyBound()), 2).andThen(
                                Commands.runOnce(() -> Robot.arm.getRotation().translateMotor(0))
                ),
                Commands.runOnce(() -> Robot.arm.getRotation().translateMotor(0))
        );
    }
}
