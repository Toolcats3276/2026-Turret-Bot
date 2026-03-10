package frc.robot.commands.BaseCommands.Infeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfeedPivotSS;

public class InfeedPID extends Command{

    private InfeedPivotSS s_InfeedPivot;
    private double setPoint;
    private double maxSpeed;


    public InfeedPID(InfeedPivotSS s_InfeedPivot, double setPoint, double maxSpeed){
        this.s_InfeedPivot = s_InfeedPivot;
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
    }
    

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        s_InfeedPivot.PID(setPoint, maxSpeed);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
