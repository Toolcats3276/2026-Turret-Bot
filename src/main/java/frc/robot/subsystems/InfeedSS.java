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
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class InfeedSS extends SubsystemBase{

    private TalonFX m_InfeedPivot;
    private TalonFX m_Infeed;
    private CANcoder e_InfeedEncoder;

    private PIDController InfeedPIDController;

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;

    private double output;
    private double setPoint;
    private double maxSpeed;

    private double InfeedVal;


    
    public InfeedSS() {
        m_Infeed = new TalonFX(RobotConstants.Infeed.Infeed_Motor);
        m_Infeed.getConfigurator().apply(Robot.ctreConfigs.leftTurretConfig);
        m_Infeed.setNeutralMode(NeutralModeValue.Brake);

        m_InfeedPivot = new TalonFX(RobotConstants.Infeed.Infeed_Rotation_Motor);
        m_InfeedPivot.getConfigurator().apply(Robot.ctreConfigs.leftTurretConfig);
        m_InfeedPivot.setNeutralMode(NeutralModeValue.Brake);

        e_InfeedEncoder = new CANcoder(RobotConstants.Infeed.Infeed_Rotation_Encoder);
        e_InfeedEncoder.getConfigurator().apply(Robot.ctreConfigs.turretCANcoderConfig);
        e_InfeedEncoder.setPosition(e_InfeedEncoder.getAbsolutePosition().getValueAsDouble());

        InfeedPIDController = new PIDController(kP, kI, kD);

        
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
                m_InfeedPivot.set(0);
                break;
            }

            case PID:{
                InfeedPIDController.reset();
                output = -MathUtil.clamp(InfeedPIDController.calculate(e_InfeedEncoder.getPosition().getValueAsDouble(), setPoint), -maxSpeed, maxSpeed);
                m_InfeedPivot.set(output);
                break;
            }

        }

        SmartDashboard.putNumber("LeftTurret Output", output);
        SmartDashboard.putNumber("LeftTurret setPoint", setPoint);
        SmartDashboard.putNumber("LeftTurret Encoder Pose", e_InfeedEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("LeftTurret AbsEncoder Pose", e_InfeedEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    public void Stop(){
        TurretMode = Mode.Stop;
    }
    
    public void PID(double setPoint, double maxSpeed){
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        InfeedPIDController.reset();
        TurretMode = Mode.PID;
    }

    public double returnSetPoint(){
        return setPoint;
    }

    public Boolean atSetPoint(){
        return InfeedPIDController.atSetpoint();
    }
    
    
}


