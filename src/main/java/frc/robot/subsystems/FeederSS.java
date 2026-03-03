package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class FeederSS extends SubsystemBase {

    private TalonFX m_Feeder;

    private AngularVelocity speed;

    public FeederSS(){
            m_Feeder = new TalonFX(RobotConstants.Feeder.Feeder_Motor, CTREConfigs.CanivoreCANbus);
            m_Feeder.getConfigurator().apply(Robot.ctreConfigs.FeederConfig);
            m_Feeder.setNeutralMode(NeutralModeValue.Coast);
    }

    public enum Mode{
        Stop,
        SetSpeed,
    }

    Mode FeederMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(FeederMode) {

            case Stop:{
                m_Feeder.set(0);
                break;
            }

            case SetSpeed:{
                m_Feeder.set(speed.in(RotationsPerSecond));
            }
        }

        // SmartDashboard.putNumber("IndexerSetSpeed", speed.in(RotationsPerSecond));
        SmartDashboard.putNumber("FeederCurrentSpeed", m_Feeder.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        FeederMode = Mode.Stop;
    }
    
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        FeederMode = Mode.SetSpeed;
    } 
}