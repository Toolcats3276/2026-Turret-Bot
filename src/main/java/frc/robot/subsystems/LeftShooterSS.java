package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class LeftShooterSS extends SubsystemBase {

    private TalonFX m_shooterBottomMotor;
    private TalonFX m_shooterTopMotor;
    private double speed;


  
    public LeftShooterSS(){
            m_shooterBottomMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Left_Motor);
            m_shooterBottomMotor.getConfigurator().apply(Robot.ctreConfigs.shooterBottomConfig);
            m_shooterBottomMotor.setNeutralMode(NeutralModeValue.Coast);

            m_shooterTopMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Right_Motor);
            m_shooterTopMotor.getConfigurator().apply(Robot.ctreConfigs.shooterTopConfig);
            m_shooterTopMotor.setNeutralMode(NeutralModeValue.Coast);
            m_shooterTopMotor.setControl(new StrictFollower(m_shooterBottomMotor.getDeviceID()));
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
                m_shooterBottomMotor.set(0);
                break;
            }

            case SetSpeed:{
                m_shooterBottomMotor.set(speed);
            }
        }

        SmartDashboard.putNumber("ShooterSetSpeed", speed);
        SmartDashboard.putNumber("ShooterCurrentSpeed", m_shooterBottomMotor.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(double speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    
    
}