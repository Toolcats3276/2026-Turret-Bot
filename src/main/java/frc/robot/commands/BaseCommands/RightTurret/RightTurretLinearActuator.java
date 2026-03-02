package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterHoodSS;

public class RightTurretLinearActuator extends Command{

    private RightShooterHoodSS s_RightShooterHood;
    private double setPoint;


    public RightTurretLinearActuator(RightShooterHoodSS s_RightShooterHood, double setPoint){
        this.s_RightShooterHood = s_RightShooterHood;
        this.setPoint = setPoint;
        addRequirements(s_RightShooterHood);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_RightShooterHood.LinearActuator(setPoint);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
