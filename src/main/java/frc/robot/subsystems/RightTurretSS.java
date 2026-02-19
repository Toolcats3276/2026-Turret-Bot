package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.vision.LimelightAssistant;

/* SEE WristSS FOR EXPLANATIONS */

public class RightTurretSS extends SubsystemBase{
    /*General Turret Rotation*/
    private TalonFX m_TurretMotor;
    private CANcoder e_TurretEncoder;

    private PIDController TurretPIDController;

    private final double kP = .23;
    private final double kI = 0;
    private final double kD = 0.003;

    private double output;
    private double setPoint;
    private double maxSpeed;
    private double ManualVal;
    private double TX;

    private double StartingDeadStop = 0.5;
    private double FinalDeadStop = 11;
    
    /*Limelight*/

    private final LimelightAssistant LeftLimelight;
    private final LimelightAssistant RightLimelight;

    private final PIDController LLRotationPidController;
        private final double LLkP = 0.15;
        private final double LLkI = 0;
        private final double LLkD = 0;
    
    public RightTurretSS() {
        /*Turret General Rotation*/
        m_TurretMotor = new TalonFX(RobotConstants.FrontRightTurret.Turret_Rotation_Motor);
        m_TurretMotor.getConfigurator().apply(Robot.ctreConfigs.RightTurretConfig);
        m_TurretMotor.setNeutralMode(NeutralModeValue.Brake);

        e_TurretEncoder = new CANcoder(RobotConstants.FrontRightTurret.Turret_Rotation_Encoder);
        e_TurretEncoder.getConfigurator().apply(Robot.ctreConfigs.RightTurretCancoderConfig);
        e_TurretEncoder.setPosition(e_TurretEncoder.getAbsolutePosition().getValueAsDouble());

        TurretPIDController = new PIDController(kP, kI, kD);

        /*Limelight*/
        LeftLimelight = new LimelightAssistant("limelight-rlt", VecBuilder.fill(0,0,0), false);
        RightLimelight = new LimelightAssistant("limelight-rrt", VecBuilder.fill(0,0,0), false);

        LLRotationPidController = new PIDController(LLkP, LLkI, LLkD);
    }

     public enum Mode{
        Stop,
        PID,
        AutoAim,
        Manual
    }

    Mode TurretMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(TurretMode) {

            case Stop:{
                m_TurretMotor.set(0);
                break;
            }

            case PID:{
                TurretPIDController.reset();
                output = -MathUtil.clamp(TurretPIDController.calculate(e_TurretEncoder.getPosition().getValueAsDouble(), setPoint), -maxSpeed, maxSpeed);
                m_TurretMotor.set(output);
                break;
            }

            case Manual:{
                if(e_TurretEncoder.getPosition().getValueAsDouble() > StartingDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < FinalDeadStop){
                output = MathUtil.clamp(ManualVal, -.1, .1);
                m_TurretMotor.set(output);
                }
                else {
                    output = 0;
                }
                break;
            }

            case AutoAim:{
                if(e_TurretEncoder.getPosition().getValueAsDouble() > StartingDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < FinalDeadStop){
                    output = MathUtil.clamp(LLRotationPidController.calculate(TxValue(), 0), -maxSpeed, maxSpeed);
                    m_TurretMotor.set(output);
                }
                else if(e_TurretEncoder.getPosition().getValueAsDouble() < FinalDeadStop){
                    if (TxValue() > 0){
                        output = MathUtil.clamp(LLRotationPidController.calculate(TxValue(), 0), -maxSpeed, maxSpeed);
                    }
                    else {
                        output = 0;
                    }
                    m_TurretMotor.set(output);
                }
                else if(e_TurretEncoder.getPosition().getValueAsDouble() > StartingDeadStop){
                    if (TxValue() < 0){
                        output = MathUtil.clamp(LLRotationPidController.calculate(TxValue(), 0), -maxSpeed, maxSpeed);
                    }
                    else {
                        output = 0;
                    }                    
                    m_TurretMotor.set(output);
                }
                else{
                    output = 0;
                    m_TurretMotor.set(output);
                }
                break;
            }

        }

        SmartDashboard.putNumber("RightTurret Output", output);
        SmartDashboard.putNumber("RightTurret setPoint", setPoint);
        SmartDashboard.putNumber("RightTurret Encoder Pose", e_TurretEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RightTurret AbsEncoder Pose", e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("RightTurret_P", kP);
        SmartDashboard.putNumber("TX RightTurret", TX);
        SmartDashboard.putBoolean("RightTurretInRange", e_TurretEncoder.getPosition().getValueAsDouble() < StartingDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() > FinalDeadStop);
        SmartDashboard.putBoolean("RightTurretInNegRange", e_TurretEncoder.getPosition().getValueAsDouble() < FinalDeadStop);
        SmartDashboard.putBoolean("RightTurretInPosRange", e_TurretEncoder.getPosition().getValueAsDouble() > StartingDeadStop);
    }
    
    public void Stop(){
        TurretMode = Mode.Stop;
    }

    public void Manual(double ManualVal){
        this.ManualVal = ManualVal;
        TurretMode = Mode.Manual;
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

    public void AutoAim(double maxSpeed){
        this.maxSpeed = maxSpeed;
        TurretMode = Mode.AutoAim;
    }

    public double TxValue(){
        TX = LeftLimelight.getTX() + RightLimelight.getTX();
        return TX;
    }
}


