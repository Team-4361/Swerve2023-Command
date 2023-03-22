// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.climber.ClimberArmSubsystem;
import frc.robot.subsystems.climber.ClimberWristSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vacuum.VacuumSubsystem;
import frc.robot.subsystems.vision.CameraSubsystem;
import frc.robot.util.camera.PhotonCameraModule;
import frc.robot.util.math.CameraQuality;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.ClimberPresets.CLIMBER_PRESET_GROUP;
import static frc.robot.Constants.FrontCamera.CAMERA_CONFIG;
import static frc.robot.Constants.TEST_MODE;
import static frc.robot.subsystems.swerve.SwerveDriveSubsystem.deadzone;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    public static SwerveDriveSubsystem swerveDrive;
    public static ClimberArmSubsystem arm;
    public static ClimberWristSubsystem wrist;
    public static VacuumSubsystem pump;
    public static CameraSubsystem camera;

    public static boolean pidControlEnabled = false; //true;

    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Call this method at the very end!
        swerveDrive = new SwerveDriveSubsystem(
                FL_MODULE,
                FR_MODULE,
                BL_MODULE,
                BR_MODULE,
                CHASSIS_SIDE_LENGTH
        );
        arm = new ClimberArmSubsystem();
        wrist = new ClimberWristSubsystem();
        pump = new VacuumSubsystem();
        camera = new CameraSubsystem(CameraQuality.VERY_FAST);//.addCamera(
                //new PhotonCameraModule(CAMERA_CONFIG)

        autoChooser.addOption("Charge Station Auto", Autos.coneMiddleChargeStationCommand());
        autoChooser.addOption("Cone Grab Auto", Autos.coneMiddleGetAdditionalCommand());

        autoChooser.setDefaultOption("Charge Station Auto", Autos.coneMiddleChargeStationCommand());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // *** IMPORTANT: Call this method at the VERY END of robotInit!!! *** //
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
        if (TEST_MODE) {
            CLIMBER_PRESET_GROUP.updateDashboard();
        }

        if (!Robot.swerveDrive.gyro.isCalibrating() && !Robot.swerveDrive.hasSetOffset) {
            Robot.swerveDrive.gyroRollOffset = Robot.swerveDrive.gyro.getRoll();
            new PrintCommand("SET ROLL OFFSET TO " + Robot.swerveDrive.gyroRollOffset).execute();
            Robot.swerveDrive.hasSetOffset = true;
        }
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
        CommandScheduler.getInstance().cancelAll();

        Robot.arm.getRotation().resetEncoder();
        Robot.arm.getExtension().resetEncoder();

        Robot.swerveDrive.hasSetOffset = false;

        Command autonomousCommand = robotContainer.getAutonomousCommand();

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
        Robot.arm.getExtension().translateMotor(deadzone(-RobotContainer.xbox.getLeftY()/2, 0.1));
        Robot.arm.getRotation().translateMotor(deadzone(-RobotContainer.xbox.getRightY(), 0.1));
    }


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
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
