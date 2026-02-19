package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightTurretSS;

public class RightTurretPID extends Command{

    private RightTurretSS s_RightTurret;
    private double setPoint;
    private double maxSpeed;


    public RightTurretPID(RightTurretSS s_RightTurret, double setPoint, double maxSpeed){
        this.s_RightTurret = s_RightTurret;
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        addRequirements(s_RightTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_RightTurret.PID(setPoint, maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
