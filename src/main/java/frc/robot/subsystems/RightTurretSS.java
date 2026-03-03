package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.vision.LimelightAssistant;


public class RightTurretSS extends SubsystemBase{
    /*General Turret Rotation*/
    private TalonFX m_TurretMotor;
    private CANcoder e_TurretEncoder;
    private Servo s_LinearActuator;

    private PIDController TurretPIDController;

    private final double kP = .278;
    private final double kI = 0;
    private final double kD = 0
    ;

    private double output;
    private double shootangle;
    private double setPoint;
    private double maxSpeed;
    private double ManualVal;
    private double TX;
    private double shotPower;
    private double shooterAngle;

    private double s_speed;
    private double s_length;
    private double s_setPoint;
    private double s_currentpos;

    private double NegativeDeadStop = -2.667;
    private double PositiveDeadStop = 2.667;
    
    /*Limelight*/

    public final LimelightAssistant LeftLimelight;
    public final LimelightAssistant RightLimelight;

    private final PIDController LLRotationPidController;
        private final double LLkP = 0.01;
        private final double LLkI = 0;
        private final double LLkD = 0;
    
    public RightTurretSS() {
        /*Turret General Rotation*/
        m_TurretMotor = new TalonFX(RobotConstants.FrontRightTurret.Turret_Rotation_Motor, CTREConfigs.CanivoreCANbus);
        m_TurretMotor.getConfigurator().apply(Robot.ctreConfigs.RightTurretConfig);
        m_TurretMotor.setNeutralMode(NeutralModeValue.Coast);

        e_TurretEncoder = new CANcoder(RobotConstants.FrontRightTurret.Turret_Rotation_Encoder, CTREConfigs.CanivoreCANbus);
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
        Manual,
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
                if(e_TurretEncoder.getPosition().getValueAsDouble() > NegativeDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < PositiveDeadStop){
                output = MathUtil.clamp(ManualVal, -.1, .1);
                m_TurretMotor.set(output);
                }
                else {
                    output = 0;
                }
                break;
            }

            case AutoAim:{
                if((LeftLimelight.getFiducialID() == 21 || LeftLimelight.getFiducialID() == 24 || LeftLimelight.getFiducialID() == 25 || LeftLimelight.getFiducialID() == 26 || LeftLimelight.getFiducialID() ==  27 || LeftLimelight.getFiducialID() == 18) || 
                   (RightLimelight.getFiducialID() == 21 || RightLimelight.getFiducialID() == 24 || RightLimelight.getFiducialID() == 25 || RightLimelight.getFiducialID() == 26 || RightLimelight.getFiducialID() ==  27 || RightLimelight.getFiducialID() == 18)){
                    if(e_TurretEncoder.getPosition().getValueAsDouble() > NegativeDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < PositiveDeadStop){
                        output = -MathUtil.clamp(LLRotationPidController.calculate(TxValue(), 0), -maxSpeed, maxSpeed);
                        m_TurretMotor.set(output);
                    }
                    else if(e_TurretEncoder.getPosition().getValueAsDouble() > PositiveDeadStop){
                        if (TxValue() > 0){
                            output = -MathUtil.clamp(LLRotationPidController.calculate(TxValue(), 0), -maxSpeed, maxSpeed);
                        }
                        else {
                            output = 0;
                        }
                        m_TurretMotor.set(output);
                    }
                    else if(e_TurretEncoder.getPosition().getValueAsDouble() < NegativeDeadStop){
                        if (TxValue() < 0){
                            output = -MathUtil.clamp(LLRotationPidController.calculate(TxValue(), 0), -maxSpeed, maxSpeed);
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
                }
                break;
            }

        }

        SmartDashboard.putNumber("RightTurret Output", output);
        SmartDashboard.putNumber("RightTurret setPoint", setPoint);
        SmartDashboard.putNumber("RightTurret Encoder Pose", e_TurretEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RightTurret AbsEncoder Pose", e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("TX RightTurret", TX);
        SmartDashboard.putBoolean("RightTurretInRange", e_TurretEncoder.getPosition().getValueAsDouble() > NegativeDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < PositiveDeadStop);
        SmartDashboard.putBoolean("RightTurretInNegRange", e_TurretEncoder.getPosition().getValueAsDouble() < NegativeDeadStop);
        SmartDashboard.putBoolean("RightTurretInPosRange", e_TurretEncoder.getPosition().getValueAsDouble() > PositiveDeadStop);
        SmartDashboard.putNumber("Right Linear Acutator Angle", s_LinearActuator.get());
        SmartDashboard.putNumber("Right Linear Actuator Setpoint", shootangle);

        returnPOS();

        LeftLimelightTargetBoolean();
        RightLimelightTargetBoolean();
        LimeLightTargetBoolean();

        TX = LeftLimelight.getTX() + RightLimelight.getTX();
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

    public double returnPOS(){
        return e_TurretEncoder.getPosition().getValueAsDouble();
    }

    public Boolean atSetPoint(){
        return TurretPIDController.atSetpoint();
    }

    public void AutoAim(double maxSpeed){
        this.maxSpeed = maxSpeed;
        TurretMode = Mode.AutoAim;
    }

    public double TxValue(){
        return TX;
    }

    public double TyValue(){
        return (LeftLimelight.getTY() + RightLimelight.getTY())/2;
    }

    public boolean LeftLimelightTargetBoolean(){
        return  LeftLimelight.getFiducialID() == 21 || LeftLimelight.getFiducialID() == 24 || LeftLimelight.getFiducialID() == 25 || LeftLimelight.getFiducialID() == 26 || LeftLimelight.getFiducialID() ==  27 || LeftLimelight.getFiducialID() == 18;
    }

    public boolean RightLimelightTargetBoolean(){
        return  RightLimelight.getFiducialID() == 21 || RightLimelight.getFiducialID() == 24 || RightLimelight.getFiducialID() == 25 || RightLimelight.getFiducialID() == 26 || RightLimelight.getFiducialID() ==  27 || RightLimelight.getFiducialID() == 18;
    }

    public boolean LimeLightTargetBoolean(){
        return LeftLimelightTargetBoolean() || RightLimelightTargetBoolean();
    }
    
}


