package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
    private Servo s_LinearActuator;

    private double shotVelocity;
    private double shotAngle;
    private LimelightAssistant center_Limmelight;

    private double speed;


  
    public LeftShooterSS(){
        m_shooterLeftMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Left_Motor, CTREConfigs.CanivoreCANbus);
        m_shooterLeftMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterLeftConfig);
        m_shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);
        
        m_shooterRightMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Right_Motor, CTREConfigs.CanivoreCANbus);
        m_shooterRightMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterRightConfig);
        m_shooterRightMotor.setNeutralMode(NeutralModeValue.Coast);
        m_shooterRightMotor.setControl(new StrictFollower(m_shooterLeftMotor.getDeviceID()));

        center_Limmelight = new LimelightAssistant("limelight-ty", VecBuilder.fill(0,0,0), false);

        s_LinearActuator = new Servo(9);
        s_LinearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);     
    }


    public enum Mode{
        Stop,
        SetSpeed,
        ShooterAutoAim,
        LinearActuator
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

            case LinearActuator:{
                s_LinearActuator.set(shotAngle);
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
        return shotVelocity;
    }

    public void LinearActuator(double shotAngle){
        this.shotAngle = shotAngle;
        ShooterMode = Mode.LinearActuator;
    }

    public double LinearActuatorSetPoint(){
        return shotAngle;
    }

    public static record ShooterSetpoints(
        Distance shotAngle,
        AngularVelocity shotVelocity,
        AngularVelocity indexerVelocity) {
        
    public ShooterSetpoints interpolate(ShooterSetpoints endValue, double t) {
      ShooterSetpoints result = new ShooterSetpoints(
        Millimeters.of(MathUtil.interpolate(shotAngle.in(Millimeters), endValue.shotAngle.in(Millimeters), t)),
        RotationsPerSecond.of(
              MathUtil.interpolate(
                  shotVelocity.in(RotationsPerSecond),
                    endValue.shotVelocity.in(RotationsPerSecond),
                    t)),
        RotationsPerSecond.of(
              MathUtil.interpolate(
                  indexerVelocity.in(RotationsPerSecond),
                    endValue.indexerVelocity.in(RotationsPerSecond),
                    t)));
      return result;
    }
    }
    
}