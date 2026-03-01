package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterHoodSS;

public class LeftTurretLinearActuator extends Command{

    private LeftShooterHoodSS s_RightShooterHood;
    private double setPoint;


    public LeftTurretLinearActuator(LeftShooterHoodSS s_RightShooterHood, double setPoint){
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
