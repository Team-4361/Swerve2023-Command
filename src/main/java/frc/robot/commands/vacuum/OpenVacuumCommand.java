package frc.robot.commands.vacuum;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class OpenVacuumCommand extends CommandBase {
    public OpenVacuumCommand(){
        addRequirements(Robot.pump);
    }
    @Override
    public void initialize(){
        Robot.pump.setSolenoid(true);
    }

    @Override
    public void end(boolean interrupted){
        Robot.pump.setSolenoid(false);
    }

    @Override
    public boolean isFinished() { return false;
    }
}
