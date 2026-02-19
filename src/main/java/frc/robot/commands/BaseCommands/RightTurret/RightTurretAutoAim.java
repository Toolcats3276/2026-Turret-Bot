package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightTurretSS;

public class RightTurretAutoAim extends Command{

    private RightTurretSS s_RightTurret;
    private double maxSpeed;


    public RightTurretAutoAim(RightTurretSS s_RightTurret, double maxSpeed){
        this.s_RightTurret = s_RightTurret;
        this.maxSpeed = maxSpeed;
        addRequirements(s_RightTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_RightTurret.AutoAim(maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
