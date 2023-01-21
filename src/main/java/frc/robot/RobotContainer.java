// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Control;
import frc.robot.commands.auto.PIDTargetCommand;

import static frc.robot.Constants.Chassis.DRIVE_DEAD_ZONE;
import static frc.robot.subsystems.SwerveDriveSubsystem.deadzone;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandJoystick xyStick = new CommandJoystick(Control.LEFT_STICK_ID);
    private final CommandJoystick zStick = new CommandJoystick(Control.RIGHT_STICK_ID);
    private final CommandXboxController xbox = new CommandXboxController(Control.XBOX_CONTROLLER_ID);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        
        Robot.swerveDrive.setDefaultCommand(Robot.swerveDrive.run(() ->  {
            Robot.swerveDrive.autoDrive(
                    deadzone(xyStick.getX(), DRIVE_DEAD_ZONE),
                    deadzone(xyStick.getY(), DRIVE_DEAD_ZONE),
                    deadzone(zStick.getTwist(), DRIVE_DEAD_ZONE)
            );
        }));
        
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        xbox.a().whileTrue(new PIDTargetCommand());

        //xbox.x().onTrue(Robot.swerveDrive.runOnce(() -> {
        //    Robot.swerveDrive.resetPosition();
        //}));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // test moving to april tag ID #1
        //return new PIDTargetCommand(1);
        return Robot.swerveDrive.resetGyroCommand()
                .andThen(
                        Robot.swerveDrive.followTrajectoryCommand(PathPlanner.loadPath("Test Path", new PathConstraints(3, 3)))
                )
                .andThen(new PIDTargetCommand());

    }
}
