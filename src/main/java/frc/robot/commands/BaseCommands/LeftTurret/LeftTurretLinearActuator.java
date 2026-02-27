package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.LeftTurretSS;

public class LeftTurretLinearActuator extends Command{

    private LeftTurretSS s_LeftTurret;
    private double setPoint;


    public LeftTurretLinearActuator(LeftTurretSS s_LeftTurret, double setPoint){
        this.s_LeftTurret = s_LeftTurret;
        this.setPoint = setPoint;
        addRequirements(s_LeftTurret);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_LeftTurret.LinearActuator(setPoint);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
