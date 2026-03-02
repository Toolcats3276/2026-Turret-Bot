package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftShooterSS;

public class LeftTurretLinearActuator extends Command{

    private LeftShooterSS s_LeftShooterSS;
    private double setPoint;


    public LeftTurretLinearActuator(LeftShooterSS s_LeftShooterSS, double setPoint){
        this.s_LeftShooterSS = s_LeftShooterSS;
        this.setPoint = setPoint;
        addRequirements(s_LeftShooterSS);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_LeftShooterSS.LinearActuator(setPoint);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
