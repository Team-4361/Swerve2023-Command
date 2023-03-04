package frc.robot.commands.swerve.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import static frc.robot.DriveConstants.PIDConstraint.*;

public class FollowTrajectory extends SequentialCommandGroup {

    public FollowTrajectory(PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(Robot.swerveDrive);

        if (resetOdometry) {
            Robot.swerveDrive.resetOdometry(trajectory.getInitialHolonomicPose());
        }
        addCommands(
                new PPSwerveControllerCommand(
                        trajectory,
                        Robot.swerveDrive::getPose,
                        DRIVE_PID.createPIDController(),
                        DRIVE_PID.createPIDController(),
                        HEADING_PID.createPIDController(),
                        Robot.swerveDrive::setChassisSpeeds,
                        Robot.swerveDrive)
        );
    }

    public FollowTrajectory(String trajectoryPath, boolean resetOdometry) {
        this(PathPlanner.loadPath(trajectoryPath, AUTO_CONSTRAINTS), resetOdometry);
    }
}
