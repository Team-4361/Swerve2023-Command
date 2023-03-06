package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import static frc.robot.Constants.AutoValues.AUTO_CONSTRAINTS;
import static frc.robot.Constants.AutoValues.AUTO_CONTROLLER;

public class TrajectoryCommand extends SequentialCommandGroup {
    public TrajectoryCommand(PathPlannerTrajectory trajectory, boolean resetOdometry) {
        if (resetOdometry) {
            addCommands(Robot.swerveDrive.resetOdometryCommand());
        }

        addCommands(new PPSwerveControllerCommand(
                trajectory,
                Robot.swerveDrive::getPose,
                Robot.swerveDrive.getSwerveChassis().getSwerveKinematics(),
                AUTO_CONTROLLER,
                AUTO_CONTROLLER,
                AUTO_CONTROLLER,
                Robot.swerveDrive.getSwerveChassis()::setStates,
                true,
                Robot.swerveDrive
        ));
    }

    public TrajectoryCommand(String pathName, boolean resetOdometry) {
        this(PathPlanner.loadPath(pathName, AUTO_CONSTRAINTS), resetOdometry);
    }
}
