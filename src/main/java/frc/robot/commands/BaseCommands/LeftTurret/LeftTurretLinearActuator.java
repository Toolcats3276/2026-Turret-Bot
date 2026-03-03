package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftShooterHoodSS;

public class LeftTurretLinearActuator extends Command{

    private LeftShooterHoodSS s_LeftShooterHood;
    private Distance setPoint;


    public LeftTurretLinearActuator(LeftShooterHoodSS s_LeftShooterHood, Distance setPoint){
        this.s_LeftShooterHood = s_LeftShooterHood;
        this.setPoint = setPoint;
        addRequirements(s_LeftShooterHood);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_LeftShooterHood.LinearActuator(setPoint);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
