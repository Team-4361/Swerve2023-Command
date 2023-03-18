// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auto.Autos;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Control;

import static frc.robot.Constants.Chassis.DRIVE_DEAD_ZONE;
import static frc.robot.Constants.ClimberPresets.CLIMBER_PRESET_GROUP;
import static frc.robot.subsystems.swerve.SwerveDriveSubsystem.deadzone;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandJoystick xyStick = new CommandJoystick(Control.LEFT_STICK_ID);
    private final CommandJoystick zStick = new CommandJoystick(Control.RIGHT_STICK_ID);
    public static final CommandXboxController xbox = new CommandXboxController(Control.XBOX_CONTROLLER_ID);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        Robot.swerveDrive.setDefaultCommand(Robot.swerveDrive.run(() ->  {
            Robot.swerveDrive.autoDrive(
                    deadzone(-xyStick.getY(), DRIVE_DEAD_ZONE),
                    deadzone(-xyStick.getX(), DRIVE_DEAD_ZONE),
                    deadzone(zStick.getTwist(), 0.20)
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
        xyStick.button(8).onTrue(Robot.swerveDrive.toggleFieldOrientedCommand());
        xyStick.button(12).onTrue(Robot.swerveDrive.resetGyroCommand());
        //xyStick.button(9).onTrue(Robot.swerveDrive.toggleClosedLoopCommand());

        //xyStick.button(10).onTrue(Commands.runOnce(()->Robot.pidControlEnabled = !Robot.pidControlEnabled));
        xyStick.trigger().or(zStick.trigger()).whileTrue(Robot.swerveDrive.holdPrecisionModeCommand());
        xyStick.button(3).whileTrue(Robot.swerveDrive.lockWheelCommand());

        xyStick.button(5).or(zStick.button(6)).onTrue(Robot.pump.toggleLEDCommand());

        ///////////////////////////////// XBOX CONTROLS

        xbox.a().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(0)));
        xbox.b().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(2)));
        xbox.y().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(3)));
        xbox.x().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(4)));

        xbox.rightBumper().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(5)));


        xbox.rightTrigger().whileTrue(Commands.runEnd(() -> {
            Robot.wrist.translateMotor(-xbox.getRightTriggerAxis()/2);
        }, () -> {
            Robot.wrist.translateMotor(0);
        }));

        xbox.leftTrigger().whileTrue(Commands.runEnd(() -> {
            Robot.wrist.translateMotor(xbox.getLeftTriggerAxis()/2);
        }, () -> {
            Robot.wrist.translateMotor(0);
        }));

        xbox.leftStick().onTrue(Commands.runOnce(() -> {
            Robot.wrist.resetEncoder();
            Robot.arm.getRotation().resetEncoder();
            Robot.arm.getExtension().resetEncoder();
        }));

        xbox.leftBumper().onTrue(Commands.runOnce(() -> Robot.pump.toggleVacuum()));
    
        xbox.povUp().onTrue(Robot.pump.openVacuumCommand());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /* 
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
            */
        //return Autos.coneMiddleGetAdditionalCommand();
        return Autos.coneMiddleGetAdditionalCommand();
    }
}