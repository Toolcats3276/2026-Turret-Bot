package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class RightShooterSS extends SubsystemBase {

    private TalonFX m_shooterLeftMotor;
    private TalonFX m_shooterRightMotor;
    private double speed;


  
    public RightShooterSS(){
            m_shooterLeftMotor = new TalonFX(RobotConstants.FrontRightTurret.Shoot_Motor_Left_Motor);
            m_shooterLeftMotor.getConfigurator().apply(Robot.ctreConfigs.RightshooterBottomConfig);
            m_shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);

            m_shooterRightMotor = new TalonFX(RobotConstants.FrontRightTurret.Shoot_Motor_Right_Motor);
            m_shooterRightMotor.getConfigurator().apply(Robot.ctreConfigs.RightshooterTopConfig);
            m_shooterRightMotor.setNeutralMode(NeutralModeValue.Coast);
            m_shooterRightMotor.setControl(new StrictFollower(m_shooterLeftMotor.getDeviceID()));
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
                m_shooterLeftMotor.set(0);
                break;
            }

            case SetSpeed:{
                m_shooterLeftMotor.set(speed);
            }
        }

        SmartDashboard.putNumber("RightShooterSetSpeed", speed);
        SmartDashboard.putNumber("RightLeftShooterCurrentSpeed", m_shooterLeftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("RightRightShooterCurrentSpeed", m_shooterRightMotor.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(double speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    
    
}