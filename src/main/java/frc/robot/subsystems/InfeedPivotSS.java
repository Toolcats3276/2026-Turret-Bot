package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class InfeedPivotSS extends SubsystemBase{

    private TalonFX m_Infeed_Pivot_Left;
    private TalonFX m_Infeed_Pivot_Right;
    private CANcoder e_InfeedEncoder;

    private PIDController InfeedPIDController;

    private final double kP = 1.5;
    private final double kI = 0;
    private final double kD = 0;

    private double output;
    private double setPoint;
    private double maxSpeed;
    private double ManualVal;
    
    public InfeedPivotSS() {
        m_Infeed_Pivot_Left = new TalonFX(RobotConstants.Infeed.Infeed_Rotation_Motor_Left, CTREConfigs.CanivoreCANbus);
        m_Infeed_Pivot_Left.getConfigurator().apply(Robot.ctreConfigs.InfeedPivotLeftConfig);
        m_Infeed_Pivot_Left.setNeutralMode(NeutralModeValue.Brake);

        m_Infeed_Pivot_Right = new TalonFX(RobotConstants.Infeed.Infeed_Rotation_Motor_Right, CTREConfigs.CanivoreCANbus);
        m_Infeed_Pivot_Right.getConfigurator().apply(Robot.ctreConfigs.InfeedPivotRightConfig);
        m_Infeed_Pivot_Right.setNeutralMode(NeutralModeValue.Brake);
        // m_Infeed_Pivot_Right.setControl(new StrictFollower(m_Infeed_Pivot_Left.getDeviceID()));

        e_InfeedEncoder = new CANcoder(RobotConstants.Infeed.Infeed_Rotation_Encoder, CTREConfigs.CanivoreCANbus);
        e_InfeedEncoder.getConfigurator().apply(Robot.ctreConfigs.InfeedCancoderConfig);
        e_InfeedEncoder.setPosition(e_InfeedEncoder.getAbsolutePosition().getValueAsDouble());

        InfeedPIDController = new PIDController(kP, kI, kD);
    }

     public enum Mode{
        Stop,
        PID,
        Manual
    }

    Mode InfeedMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(InfeedMode) {

            case Stop:{
                m_Infeed_Pivot_Left.set(0);
                m_Infeed_Pivot_Right.set(0);
                break;
            }

            case Manual:{
                output = MathUtil.clamp(ManualVal, -1, 1);
                m_Infeed_Pivot_Left.set(output);
                break;
            }

            case PID:{
                InfeedPIDController.reset();
                output = MathUtil.clamp(InfeedPIDController.calculate(e_InfeedEncoder.getPosition().getValueAsDouble(), setPoint), -maxSpeed, maxSpeed);
                m_Infeed_Pivot_Left.set(output);
                m_Infeed_Pivot_Right.set(output);
                
                break;
            }

        }

        SmartDashboard.putNumber("Infeed Pivot Output", output);
        SmartDashboard.putNumber("Infeed Pivot setPoint", setPoint);
        SmartDashboard.putNumber("Infeed Pivot Encoder Pose", e_InfeedEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Infeed Pivot AbsEncoder Pose", e_InfeedEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Infeed Left Velocity", m_Infeed_Pivot_Left.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Infeed Right Velocity", m_Infeed_Pivot_Right.getVelocity().getValueAsDouble());
    }
    
    public void Stop(){
        InfeedMode = Mode.Stop;
    }

     public void Manual(double ManualVal){
        this.ManualVal = ManualVal;
        InfeedMode = Mode.Manual;
    }
    
    public void PID(double setPoint, double maxSpeed){
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        InfeedPIDController.reset();
        InfeedMode = Mode.PID;
    }

    public double returnSetPoint(){
        return setPoint;
    }

    public Boolean atSetPoint(){
        return InfeedPIDController.atSetpoint();
    }  
    
}


