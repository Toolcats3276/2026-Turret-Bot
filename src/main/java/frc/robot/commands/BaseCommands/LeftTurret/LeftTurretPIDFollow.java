package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightTurretSS;

public class LeftTurretPIDFollow extends Command{

    private RightTurretSS s_RightTurret;
    private LeftTurretSS s_LeftTurret;
    private double maxSpeed;


    public LeftTurretPIDFollow(RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret, double maxSpeed){
        this.s_RightTurret = s_RightTurret;
        this.s_LeftTurret = s_LeftTurret;
        this.maxSpeed = maxSpeed;
        addRequirements(s_RightTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_LeftTurret.PID(s_RightTurret.returnPOS(), maxSpeed);
        s_RightTurret.AutoAim(maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
