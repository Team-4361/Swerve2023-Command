// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
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
import frc.robot.util.math.CameraQuality;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.ClimberPresets.*;
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
    public static PowerDistribution power;

    public static boolean pidControlEnabled = true; //true;
    public static boolean limitSwitchBypass = false; //false;

    public static SendableChooser<Integer> autoMode = new SendableChooser<>();

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
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
        camera = new CameraSubsystem(CameraQuality.VERY_FAST);
        power = new PowerDistribution();

        autoMode.addOption("CHARGE YES", 2);
        autoMode.addOption("CHARGE NO", 1);
        autoMode.addOption("HIGH ONLY", 0);
        autoMode.addOption("EXPERIMENT", 3);

        autoMode.setDefaultOption("CHARGE YES", 2);

        SmartDashboard.putData("Auto Chooser", autoMode);

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

        if (!Robot.swerveDrive.gyro.isCalibrating() && !Robot.swerveDrive.hasSetOffset) {
            Robot.swerveDrive.gyroRollOffset = Robot.swerveDrive.gyro.getRoll();
            new PrintCommand("SET ROLL OFFSET TO " + Robot.swerveDrive.gyroRollOffset).execute();
            Robot.swerveDrive.hasSetOffset = true;
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        Robot.arm.getRotation().resetEncoder();
        Robot.arm.getExtension().resetEncoder();

        // Force the gyroscope pitch offset to be reset. Used for auto-balance capabilities.
        Robot.swerveDrive.hasSetOffset = false;

        switch (autoMode.getSelected()) {
            case 0: Autos.AutoCommand.highDropOnlyCommand().schedule(); break;
            case 1: Autos.AutoCommand.highCubeNoStationCommand().schedule(); break;
            case 2: Autos.AutoCommand.highCubeAutoBalanceCommand().schedule(); break;
            case 3: Autos.AutoCommand.highCubeAdditionalCommand().schedule(); break;
            default: break;
        }
    }

    @Override public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void teleopInit() { CommandScheduler.getInstance().cancelAll(); }

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Robot.arm.getExtension().translateMotor(deadzone(-RobotContainer.xbox.getLeftY() / 2, 0.1));
        Robot.arm.getRotation().translateMotor(deadzone(-RobotContainer.xbox.getRightY(), 0.1));
    }
}