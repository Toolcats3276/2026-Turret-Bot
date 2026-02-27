package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightTurretSS;

public class RightTurretLinearActuator extends Command{

    private RightTurretSS s_RightTurret;
    private double setPoint;


    public RightTurretLinearActuator(RightTurretSS s_RightTurret, double setPoint){
        this.s_RightTurret = s_RightTurret;
        this.setPoint = setPoint;
        addRequirements(s_RightTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
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
