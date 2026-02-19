package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftTurretSS;

public class LeftTurretAutoAim extends Command{

    private LeftTurretSS s_LeftTurret;
    private double maxSpeed;


    public LeftTurretAutoAim(LeftTurretSS s_LeftTurret, double maxSpeed){
        this.s_LeftTurret = s_LeftTurret;
        this.maxSpeed = maxSpeed;
        addRequirements(s_LeftTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_LeftTurret.AutoAim(maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
