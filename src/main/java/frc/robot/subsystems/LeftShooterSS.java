package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.CTREConfigs;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.vision.LimelightAssistant;

/* SEE WristSS FOR EXPLANATIONS */

public class LeftShooterSS extends SubsystemBase {

    // private TalonFX m_shooterRightMotor;
    private TalonFX m_shooterLeftMotor;

    private final StatusSignal<AngularAcceleration> shotAcceleration;

    private AngularVelocity speed;
    private AngularVelocity ShootVelocity;
    private AngularVelocity Shooter_Tolerance = RotationsPerSecond.of(.0015);

    private final LimelightAssistant LeftLimelight;
    private final LimelightAssistant RightLimelight;

    private double TYVALUE;

    private final VelocityVoltage VelocityVoltage;
      
        public LeftShooterSS(){
            m_shooterLeftMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Left_Motor, CTREConfigs.CanivoreCANbus);
            m_shooterLeftMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterLeftConfig);
            m_shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);
            
            // m_shooterRightMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor_Right_Motor, CTREConfigs.CanivoreCANbus);
            // m_shooterRightMotor.getConfigurator().apply(Robot.ctreConfigs.LeftshooterRightConfig);
            // m_shooterRightMotor.setNeutralMode(NeutralModeValue.Coast);
            // m_shooterRightMotor.setControl(new StrictFollower(m_shooterLeftMotor.getDeviceID()));
       
            shotAcceleration = m_shooterLeftMotor.getAcceleration();

            VelocityVoltage = new VelocityVoltage(0);
    
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
                    // m_shooterLeftMotor.set(speed.in(RotationsPerSecond));
                    m_shooterLeftMotor.setControl(VelocityVoltage.withVelocity(speed.in(RotationsPerSecond)));
                     break;
                }
                
            }
            
            
            
            // SmartDashboard.putNumber("LeftShooterSetSpeed", speed.in(RotationsPerSecond));
            // SmartDashboard.putNumber("LeftRightShooterCurrentSpeed", m_shooterRightMotor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("LeftLeftShooterCurrentSpeed", m_shooterLeftMotor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Left Turret TY", TyValue());
            SmartDashboard.putNumber("Left Shooter Tolerance", Shooter_Tolerance.in(RotationsPerSecond));
            
            TyValue();
    
            
            LeftLimelightTargetBoolean();
            RightLimelightTargetBoolean();
            LimeLightTargetBoolean();
        }
    
        public void Stop(){
            ShooterMode = Mode.Stop;
        }

        public AngularVelocity shootervelocity(){
            return m_shooterLeftMotor.getVelocity().getValue();
        }
        
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    public double TyValue(){
        if (LeftLimelightTargetBoolean()) {
            TYVALUE = LeftLimelight.getTY();
        }
        else if (RightLimelightTargetBoolean()) {
            TYVALUE = RightLimelight.getTY();
        }
        else if (RightLimelightTargetBoolean() && LeftLimelightTargetBoolean()) {
            TYVALUE = (LeftLimelight.getTY() + RightLimelight.getTY())/2;
        }

        return TYVALUE;
    }

    public boolean LeftLimelightTargetBoolean(){
        return  LeftLimelight.getFiducialID() == 21 || 
                LeftLimelight.getFiducialID() == 24 || 
                LeftLimelight.getFiducialID() == 25 || 
                LeftLimelight.getFiducialID() == 26 || 
                LeftLimelight.getFiducialID() == 27 || 
                LeftLimelight.getFiducialID() == 18 || 
                LeftLimelight.getFiducialID() == 19 ||
                LeftLimelight.getFiducialID() == 20 ||
                LeftLimelight.getFiducialID() == 5 || 
                LeftLimelight.getFiducialID() == 8 || 
                LeftLimelight.getFiducialID() == 9 || 
                LeftLimelight.getFiducialID() == 10 || 
                LeftLimelight.getFiducialID() == 11 ||
                LeftLimelight.getFiducialID() == 12 || 
                LeftLimelight.getFiducialID() == 2 || 
                LeftLimelight.getFiducialID() == 3 || 
                LeftLimelight.getFiducialID() == 4;
    }

    public boolean RightLimelightTargetBoolean(){
        return  RightLimelight.getFiducialID() == 21 || 
                RightLimelight.getFiducialID() == 24 || 
                RightLimelight.getFiducialID() == 25 || 
                RightLimelight.getFiducialID() == 27 || 
                RightLimelight.getFiducialID() == 26 || 
                RightLimelight.getFiducialID() == 18 || 
                RightLimelight.getFiducialID() == 19 ||
                RightLimelight.getFiducialID() == 20 ||
                RightLimelight.getFiducialID() == 5 || 
                RightLimelight.getFiducialID() == 8 || 
                RightLimelight.getFiducialID() == 9 || 
                RightLimelight.getFiducialID() == 10 || 
                RightLimelight.getFiducialID() == 11 || 
                RightLimelight.getFiducialID() == 12 ||
                RightLimelight.getFiducialID() == 2 || 
                RightLimelight.getFiducialID() == 3 || 
                RightLimelight.getFiducialID() == 4;
    }

    public boolean LimeLightTargetBoolean(){
        return LeftLimelightTargetBoolean() || RightLimelightTargetBoolean();
    }

    public static record LeftShooterSetpoints(
        Distance shotAngle,
        AngularVelocity shotVelocity) {

        
        
    public LeftShooterSetpoints interpolate(LeftShooterSetpoints endValue, double t) {
      LeftShooterSetpoints result = new LeftShooterSetpoints(
        Millimeters.of(MathUtil.interpolate(shotAngle.in(Millimeters), endValue.shotAngle.in(Millimeters), t)),
        RotationsPerSecond.of(
              MathUtil.interpolate(
                  shotVelocity.in(RotationsPerSecond),
                    endValue.shotVelocity.in(RotationsPerSecond),
                    t))
        );
      return result;
    }
    }
    
}