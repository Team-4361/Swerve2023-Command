// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.camera.PhotonCameraModule;
import frc.robot.subsystems.vision.CameraSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.climber.ClimberArmSubsystem;
import frc.robot.subsystems.fourbar.FourBarArmSubsystem;
import frc.robot.subsystems.fourbar.FourBarGripperSubsystem;
import frc.robot.subsystems.fourbar.FourBarWristSubsystem;

import static frc.robot.Constants.FrontCamera.*;
import static frc.robot.Constants.FrontCamera.CAMERA_NAME;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand, testCommand;
    private RobotContainer robotContainer;

    public static SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem();
    public static CameraSubsystem cameras = new CameraSubsystem();
    public static ClimberArmSubsystem climberSubsystem = new ClimberArmSubsystem();
    public static FourBarArmSubsystem armSubsystem = new FourBarArmSubsystem();
    public static FourBarGripperSubsystem gripperSubsystem = new FourBarGripperSubsystem();
    public static FourBarWristSubsystem wristSubsystem = new FourBarWristSubsystem();

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Call this method at the very end!
        swerveDrive = new SwerveDriveSubsystem();
        cameras = new CameraSubsystem(new PhotonCameraModule(CAMERA_CONFIG));

        robotContainer = new RobotContainer();
    }


    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }


    @Override
    public void disabledPeriodic() {
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }


    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        testCommand = robotContainer.getTestCommand();

        // schedule the autonomous command (example)
        if (testCommand != null) {
            testCommand.schedule();
        }
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }


    /**
     * This method is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }


    /**
     * This method is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
