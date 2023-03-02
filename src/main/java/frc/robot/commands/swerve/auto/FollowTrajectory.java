package frc.robot.commands.swerve.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.DriveConstants.PID.*;

public class FollowTrajectory extends SequentialCommandGroup {

    public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(drivebase);

        if (resetOdometry) {
            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        }

        addCommands(
                new PPSwerveControllerCommand(
                        trajectory,
                        drivebase::getPose,
                        X_AUTO_PID.createPIDController(),
                        Y_AUTO_PID.createPIDController(),
                        ANGLE_AUTO_PID.createPIDController(),
                        drivebase::setChassisSpeeds,
                        drivebase)
        );
    }
}
