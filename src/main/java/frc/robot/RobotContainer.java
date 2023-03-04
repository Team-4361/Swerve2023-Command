// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Control;
import frc.robot.commands.swerve.auto.Autos;
import frc.robot.commands.swerve.drivebase.TeleopDrive;

import static frc.robot.Robot.deadband;


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
        if (RobotBase.isSimulation()) {
            simButtonBindings();
        } else {
            configureBindings();
        }
    }

    private void simButtonBindings() {
        Robot.swerveDrive.setDefaultCommand(
                new TeleopDrive(
                        Robot.swerveDrive,
                        () -> -deadband(xbox.getLeftY(), 0.05),
                        () -> -deadband(xbox.getLeftX(), 0.05),
                        () -> deadband(xbox.getRightX(), 0.05),
                        () -> Robot.swerveDrive.isFieldOriented(),
                        () -> Robot.swerveDrive.isOpenLoop(),
                        false
                )
        );

        xbox.x().onTrue(Commands.runOnce(() -> Robot.swerveDrive.resetOdometry(new Pose2d())));
        xbox.y().onTrue(Robot.swerveDrive.toggleFieldOrientedCommand());
        xbox.a().onTrue(Robot.swerveDrive.resetGyroCommand());
        xbox.b().onTrue(Robot.swerveDrive.toggleOpenLoopCommand());
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
        xyStick.button(9).onTrue(Robot.swerveDrive.toggleOpenLoopCommand());

        //xyStick.button(10).onTrue(Commands.runOnce(()->Robot.pidControlEnabled = !Robot.pidControlEnabled));
        xyStick.trigger().or(zStick.trigger()).whileTrue(Robot.swerveDrive.holdPrecisionModeCommand());
        xyStick.button(3).whileTrue(Robot.swerveDrive.lockWheelCommand());

        //xbox.a().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(0)));
        //xbox.b().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(1)));
        //xbox.y().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(2)));
        //xbox.rightBumper().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setCurrentPreset(3)));

        Robot.swerveDrive.setDefaultCommand(
                new TeleopDrive(
                        Robot.swerveDrive,
                        () -> -deadband(xyStick.getY(), 0.05),
                        () -> -deadband(xyStick.getX(), 0.05),
                        () -> deadband(zStick.getTwist(), 0.05),
                        () -> Robot.swerveDrive.isFieldOriented(),
                        () -> Robot.swerveDrive.isOpenLoop(),
                        false
                )
        );

        xbox.x().onTrue(Commands.runOnce(() -> Robot.swerveDrive.resetGyro()));

        xbox.rightTrigger().whileTrue(Commands.runEnd(
                () -> Robot.wrist.translateMotor(-xbox.getRightTriggerAxis()/2),
                () -> Robot.wrist.translateMotor(0)
        ));

        xbox.leftTrigger().whileTrue(Commands.runEnd(
                () -> Robot.wrist.translateMotor(xbox.getLeftTriggerAxis()/2),
                () -> Robot.wrist.translateMotor(0)
        ));

        xbox.leftStick().onTrue(Commands.runOnce(() -> {
            Robot.wrist.resetEncoder();
            Robot.arm.getRotation().resetEncoder();
            Robot.arm.getExtension().resetEncoder();
        }));

        xyStick.button(7).whileTrue(Commands.runEnd(
                () -> Robot.pump.setSolenoid(DoubleSolenoid.Value.kForward),
                () -> Robot.pump.setSolenoid(DoubleSolenoid.Value.kOff)
        ));

        xyStick.button(11).whileTrue(Commands.runEnd(
                () -> Robot.pump.setSolenoid(DoubleSolenoid.Value.kReverse),
                () -> Robot.pump.setSolenoid(DoubleSolenoid.Value.kOff)
        ));

        xbox.leftBumper().whileTrue(Robot.pump.runEnd(
                () -> Robot.pump.activate(0.45),
                () -> Robot.pump.deactivate()
        ));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Autos.driveAndSpin(Robot.swerveDrive);
    }
}