package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class IndexerSS extends SubsystemBase {

    private TalonFX m_Indexer_Left;
    private TalonFX m_Indexer_Right;

    private AngularVelocity speed;

    public IndexerSS(){
            m_Indexer_Left = new TalonFX(RobotConstants.Indexer.Indexer_Motor_Left, CTREConfigs.CanivoreCANbus);
            m_Indexer_Left.getConfigurator().apply(Robot.ctreConfigs.IndexerLeftConfig);
            m_Indexer_Left.setNeutralMode(NeutralModeValue.Coast);

            m_Indexer_Right = new TalonFX(RobotConstants.Indexer.Indexer_Motor_Right, CTREConfigs.CanivoreCANbus);
            m_Indexer_Right.getConfigurator().apply(Robot.ctreConfigs.IndexerRightConfig);
            m_Indexer_Right.setNeutralMode(NeutralModeValue.Coast);
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
                m_Indexer_Left.set(0);
                break;
            }

            case SetSpeed:{
                m_Indexer_Left.set(speed.in(RotationsPerSecond));
            }
        }

        // SmartDashboard.putNumber("IndexerSetSpeed", speed.in(RotationsPerSecond));
        SmartDashboard.putNumber("IndexerCurrentSpeed", m_Indexer_Left.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        IndexerMode = Mode.Stop;
    }
    
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        IndexerMode = Mode.SetSpeed;
    } 
}