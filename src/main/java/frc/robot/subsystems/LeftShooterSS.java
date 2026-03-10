package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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

    private final StatusSignal<AngularVelocity> shotVelocity;
    private final StatusSignal<AngularAcceleration> shotAcceleration;

    private AngularVelocity speed;
    private AngularVelocity Shooter_Tolerance = RotationsPerSecond.of(.015);

    private final LimelightAssistant LeftLimelight;
    private final LimelightAssistant RightLimelight;


  
    public LeftShooterSS(){
        m_shooterLeftMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Left_Motor, CTREConfigs.CanivoreCANbus);
        m_shooterLeftMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterLeftConfig);
        m_shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);
        
        m_shooterRightMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Right_Motor, CTREConfigs.CanivoreCANbus);
        m_shooterRightMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterRightConfig);
        m_shooterRightMotor.setNeutralMode(NeutralModeValue.Coast);
        m_shooterRightMotor.setControl(new StrictFollower(m_shooterLeftMotor.getDeviceID()));
   
        shotVelocity = m_shooterLeftMotor.getVelocity();
        shotAcceleration = m_shooterLeftMotor.getAcceleration();

        LeftLimelight = new LimelightAssistant("limelight-llt", VecBuilder.fill(0,0,0), false);
        RightLimelight = new LimelightAssistant("limelight-lrt", VecBuilder.fill(0,0,0), false);
    }


    public enum Mode{
        Stop,
        SetSpeed
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
                m_shooterLeftMotor.set(speed.in(RotationsPerSecond));
            }

            BaseStatusSignal.refreshAll(shotVelocity);

        }

        

        // SmartDashboard.putNumber("LeftShooterSetSpeed", speed.in(RotationsPerSecond));
        SmartDashboard.putNumber("LeftRightShooterCurrentSpeed", m_shooterRightMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("LeftLeftShooterCurrentSpeed", m_shooterLeftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Left Ready to shoot", isReadyToShoot());
        SmartDashboard.putNumber("Left Turret TY", TyValue());

        TyValue();
    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    public double TyValue(){
        return (LeftLimelight.getTY() + RightLimelight.getTY())/2;
    }

    public boolean isReadyToShoot(){
        return MathUtil.isNear(shotVelocity.getValueAsDouble(), m_shooterLeftMotor.getVelocity().getValueAsDouble(), Shooter_Tolerance.in(RotationsPerSecond));
    }

    public static record LeftShooterSetpoints(
        Distance shotAngle,
        AngularVelocity shotVelocity,
        AngularVelocity indexerVelocity,
        AngularVelocity feederVelocity,
        AngularVelocity infeedVelocity) {
        
    public LeftShooterSetpoints interpolate(LeftShooterSetpoints endValue, double t) {
      LeftShooterSetpoints result = new LeftShooterSetpoints(
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
                    t)),
        RotationsPerSecond.of(
              MathUtil.interpolate(
                  feederVelocity.in(RotationsPerSecond),   
                    endValue.feederVelocity.in(RotationsPerSecond), 
                    t)),
        RotationsPerSecond.of(
              MathUtil.interpolate(
                  infeedVelocity.in(RotationsPerSecond),
                    endValue.infeedVelocity.in(RotationsPerSecond),
                    t))   
        );
      return result;
    }
    }
    
}