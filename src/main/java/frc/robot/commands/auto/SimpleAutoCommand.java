package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

import static frc.robot.Constants.Chassis.CHASSIS_MAX_SPEED;

public class SimpleAutoCommand extends SequentialCommandGroup {
    public SimpleAutoCommand() {
        super(
                Robot.pump.run(() -> Robot.pump.activate(0.45)),
                Robot.swerveDrive.runOnce(() -> Robot.swerveDrive.drive(
                        new ChassisSpeeds(-CHASSIS_MAX_SPEED/2, 0, 0)
                )),
                new WaitCommand(3),
                Robot.swerveDrive.runOnce(() -> Robot.swerveDrive.drive(new ChassisSpeeds()))
        );
    }
}
