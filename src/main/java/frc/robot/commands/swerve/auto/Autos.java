// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

import static frc.robot.DriveConstants.PIDConstraint.DRIVE_PID;
import static frc.robot.DriveConstants.PIDConstraint.HEADING_PID;

public final class Autos {

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static CommandBase driveAndSpin(SwerveSubsystem swerve) {
        return Commands.sequence(
                new RepeatCommand(new InstantCommand(() -> swerve.drive(new Translation2d(1, 0), 5, true, true), swerve)));
    }

    /**
     * Example static factory for an autonomous command.
     */
    public static CommandBase exampleAuto(SwerveSubsystem swerve) {
        List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup("SamplePath", new PathConstraints(4, 3));
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
        // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerve::getPose,
// Pose2d supplier
                swerve::resetOdometry,
// Pose2d consumer, used to reset odometry at the beginning of auto
                new PIDConstants(DRIVE_PID.p, DRIVE_PID.i, DRIVE_PID.d),
// PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(HEADING_PID.p, HEADING_PID.i, HEADING_PID.d),
// PID constants to correct for rotation error (used to create the rotation controller)
                swerve::setChassisSpeeds,
// Module states consumer used to output to the drive subsystem
                eventMap,
                false,
// Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                swerve
// The drive subsystem. Used to properly set the requirements of path following commands
        );
        return Commands.sequence(autoBuilder.fullAuto(example1));
        //    swerve.postTrajectory(example);
    }
}
