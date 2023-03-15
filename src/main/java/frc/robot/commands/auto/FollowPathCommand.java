package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import static frc.robot.Constants.AutoValues.*;

public class FollowPathCommand extends SequentialCommandGroup {

    public FollowPathCommand(PathPlannerTrajectory trajectory) {
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                Robot.swerveDrive::getPose, // Pose2d supplier
                Robot.swerveDrive::resetPositionPose, // Pose2d consumer, used to reset odometry at the beginning of auto
                Robot.swerveDrive.getSwerveChassis().getSwerveKinematics(), // SwerveDriveKinematics
                X_CONSTANTS,
                Y_CONSTANTS,
                Robot.swerveDrive.getSwerveChassis()::setStates, // Module states consumer used to output to the drive subsystem
                AUTO_EVENT_MAP,
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                Robot.swerveDrive // The drive subsystem. Used to properly set the requirements of path following commands
        );

        addCommands(autoBuilder.fullAuto(trajectory));
    }

    public FollowPathCommand(String trajectory) {
        this(PathPlanner.loadPath(trajectory, AUTO_CONSTRAINTS));
    }
}
