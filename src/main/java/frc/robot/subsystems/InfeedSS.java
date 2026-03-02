package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class InfeedSS extends SubsystemBase{

    private TalonFX m_Infeed;
    private double speed;
    
    public InfeedSS() {
        m_Infeed = new TalonFX(RobotConstants.Infeed.Infeed_Motor, CTREConfigs.CanivoreCANbus);
        m_Infeed.getConfigurator().apply(Robot.ctreConfigs.InfeedConfig);
        m_Infeed.setNeutralMode(NeutralModeValue.Brake);
        
    }

     public enum Mode{
        Stop,
        SetInfeedSpeed
    }

    Mode InfeedMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(InfeedMode) {

            case Stop:{
                m_Infeed.set(0);
                break;
            }

            case SetInfeedSpeed:{
                m_Infeed.set(speed);
                break;
            }

        }

        SmartDashboard.putNumber("Infeed setspeed", speed);
        SmartDashboard.putNumber("Infeed Velocity", m_Infeed.getRotorVelocity().getValueAsDouble());

    }
    
    public void Stop(){
        InfeedMode = Mode.Stop;
    }

    public void SetSpeed(double speed){
        this.speed = speed;
        InfeedMode = Mode.SetInfeedSpeed;
    } 
}


