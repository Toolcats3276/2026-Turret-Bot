package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.vision.LimelightAssistant;
import frc.robot.CTREConfigs;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class LeftShooterSS extends SubsystemBase {

    private TalonFX m_shooterRightMotor;
    private TalonFX m_shooterLeftMotor;
    private double shotPower;
    private double ShooterAuto;
    private LimelightAssistant center_Limmelight;

    private double speed;


  
    public LeftShooterSS(){
            m_shooterRightMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Left_Motor, CTREConfigs.CanivoreCANbus);
            m_shooterRightMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterLeftConfig);
            m_shooterRightMotor.setNeutralMode(NeutralModeValue.Coast);
            

            m_shooterLeftMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Left_Motor, CTREConfigs.CanivoreCANbus);
            m_shooterLeftMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterLeftConfig);
            m_shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);
            m_shooterLeftMotor.setControl(new StrictFollower(m_shooterLeftMotor.getDeviceID()));

            center_Limmelight = new LimelightAssistant("limelight-ty", VecBuilder.fill(0,0,0), false);

            
    }


    public enum Mode{
        Stop,
        SetSpeed,
        ShooterAutoAim
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

            case ShooterAutoAim:{
                shotPower = ((((7.49052) * Math.pow(10, -7)) * (Math.pow(center_Limmelight.getTY(), 4))) - ((0.0000363256)*(Math.pow(center_Limmelight.getTY(), 3)))+((0.000564661)*(Math.pow(center_Limmelight.getTY(), 2)))+((0.00128608)*(center_Limmelight.getTY())) + 0.522387);
                break;            
            }
        }

        SmartDashboard.putNumber("LeftShooterSetSpeed", speed);
        SmartDashboard.putNumber("LeftRightShooterCurrentSpeed", m_shooterRightMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("LeftLeftShooterCurrentSpeed", m_shooterLeftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("LeftShootPower", ShootPower());

    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(double speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    public double ShootPower(){   
        ShooterMode = Mode.ShooterAutoAim;
        return shotPower;
    }
    
}