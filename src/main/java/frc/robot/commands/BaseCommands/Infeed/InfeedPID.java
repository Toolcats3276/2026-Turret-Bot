package frc.robot.commands.BaseCommands.Infeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfeedSS;

public class InfeedPID extends Command{

    private InfeedSS s_Infeed;
    private double setPoint;
    private double maxSpeed;


    public InfeedPID(InfeedSS s_Infeed, double setPoint, double maxSpeed){
        this.s_Infeed = s_Infeed;
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        addRequirements(s_Infeed);
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_Infeed.PID(setPoint, maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
