package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FrontLeftTurret;

/* SEE WristSS FOR EXPLANATIONS */

public class LeftTurretSS extends SubsystemBase{

    private TalonFX m_Turret;
    private CANcoder e_TurretEncoder;

    private PIDController TurretPIDController;

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;

    private double output;
    private double setPoint;
    private double maxSpeed;

    private double turretVal;


    
    public LeftTurretSS() {
        m_Turret = new TalonFX(FrontLeftTurret.Turret_Rotation);
        m_Turret.getConfigurator().apply(Robot.ctreConfigs.leftTurretConfig);
        m_Turret.setNeutralMode(NeutralModeValue.Brake);

        e_TurretEncoder = new CANcoder(FrontLeftTurret.Turret_Rotation_Encoder);
        e_TurretEncoder.getConfigurator().apply(Robot.ctreConfigs.turretCANcoderConfig);
        e_TurretEncoder.setPosition(e_TurretEncoder.getAbsolutePosition().getValueAsDouble());

        TurretPIDController = new PIDController(kP, kI, kD);

        
    }

     public enum Mode{
        Stop,
        PID,
        AutoAim,

    }

    Mode TurretMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(TurretMode) {

            case Stop:{
                m_Turret.set(0);
                break;
            }

            case PID:{
                TurretPIDController.reset();
                output = -MathUtil.clamp(TurretPIDController.calculate(e_TurretEncoder.getPosition().getValueAsDouble(), setPoint), -maxSpeed, maxSpeed);
                m_Turret.set(output);
                break;
            }

        }

        SmartDashboard.putNumber("LeftTurret Output", output);
        SmartDashboard.putNumber("LeftTurret setPoint", setPoint);
        SmartDashboard.putNumber("LeftTurret Encoder Pose", e_TurretEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("LeftTurret AbsEncoder Pose", e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    public void Stop(){
        TurretMode = Mode.Stop;
    }
    
    public void PID(double setPoint, double maxSpeed){
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        TurretPIDController.reset();
        TurretMode = Mode.PID;
    }

    public double returnSetPoint(){
        return setPoint;
    }

    public Boolean atSetPoint(){
        return TurretPIDController.atSetpoint();
    }
    
    
}


