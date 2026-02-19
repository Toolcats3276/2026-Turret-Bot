package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftTurretSS;

public class LeftTurretPID extends Command{

    private LeftTurretSS s_LeftTurret;
    private double setPoint;
    private double maxSpeed;


    public LeftTurretPID(LeftTurretSS s_LeftTurret, double setPoint, double maxSpeed){
        this.s_LeftTurret = s_LeftTurret;
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        addRequirements(s_LeftTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_LeftTurret.PID(setPoint, maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
