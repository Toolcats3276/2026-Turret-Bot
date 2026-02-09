package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class IndexerSS extends SubsystemBase {

    private TalonFX m_BeltSystemMotor;
    private double speed;


  
    public IndexerSS(){
            m_BeltSystemMotor = new TalonFX(RobotConstants.Indexer.Indexer_Motor);
            m_BeltSystemMotor.getConfigurator().apply(Robot.ctreConfigs.shooterBottomConfig);
            m_BeltSystemMotor.setNeutralMode(NeutralModeValue.Coast);

    }


    public enum Mode{
        Stop,
        SetSpeed,
    }

    Mode ShooterMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(ShooterMode) {

            case Stop:{
                m_BeltSystemMotor.set(0);
                break;
            }

            case SetSpeed:{
                m_BeltSystemMotor.set(speed);
            }
        }

        SmartDashboard.putNumber("ShooterSetSpeed", speed);
        SmartDashboard.putNumber("ShooterCurrentSpeed", m_BeltSystemMotor.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(double speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    
    
}