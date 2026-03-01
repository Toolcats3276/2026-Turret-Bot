package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightTurretSS;

public class RightTurretCompliance extends Command{

    private RightTurretSS s_RightTurret;
    private double maxSpeed;
    private double setPoint;


    public RightTurretCompliance(RightTurretSS s_RightTurret, double maxSpeed, double setPoint){
        this.s_RightTurret = s_RightTurret;
        this.maxSpeed = maxSpeed;
        this.setPoint = setPoint;
        addRequirements(s_RightTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_RightTurret.AutoAim(maxSpeed);
        s_RightTurret.LinearActuator(setPoint);

    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
