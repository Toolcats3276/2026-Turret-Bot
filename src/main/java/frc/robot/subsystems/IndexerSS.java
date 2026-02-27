package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class IndexerSS extends SubsystemBase {

    private TalonFX m_Indexer;
    private double speed;

    public IndexerSS(){
            m_Indexer = new TalonFX(RobotConstants.Indexer.Indexer_Motor, CTREConfigs.CanivoreCANbus);
            m_Indexer.getConfigurator().apply(Robot.ctreConfigs.IndexerConfig);
            m_Indexer.setNeutralMode(NeutralModeValue.Coast);
    }


    public enum Mode{
        Stop,
        SetSpeed,
    }

    Mode IndexerMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(IndexerMode) {

            case Stop:{
                m_Indexer.set(0);
                break;
            }

            case SetSpeed:{
                m_Indexer.set(speed);
            }
        }

        SmartDashboard.putNumber("IndexerSetSpeed", speed);
        SmartDashboard.putNumber("IndexerCurrentSpeed", m_Indexer.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        IndexerMode = Mode.Stop;
    }
    
    public void setSpeed(double speed){
        this.speed = speed;
        IndexerMode = Mode.SetSpeed;
    } 
}