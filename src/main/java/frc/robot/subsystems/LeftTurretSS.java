package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.CTREConfigs;
import frc.robot.Constants.RobotConstants;
import frc.robot.vision.LimelightAssistant;


public class LeftTurretSS extends SubsystemBase{
    /*General Turret Rotation*/
    private TalonFX m_TurretMotor;
    private CANcoder e_TurretEncoder;

    private PIDController TurretPIDController;

    private final double kP = 0.12;
    private final double kI = 0.002;
    private final double kD = 0.000;

    private double output;
    private double setPoint;
    private double maxSpeed;
    private double ManualVal;
    private double TX;

    private double NegativeDeadStop = -3.2;
    private double PositiveDeadStop = 3.2;
    
    /*Limelight*/

    private final LimelightAssistant LeftLimelight;
    private final LimelightAssistant RightLimelight;


    private final PIDController LLRotationPidController;
        private final double LLkP = 0.004;
        private final double LLkI = 0;
        private final double LLkD = 0;
    
    public LeftTurretSS() {
        /*Turret General Rotation*/
        m_TurretMotor = new TalonFX(RobotConstants.FrontLeftTurret.Turret_Rotation_Motor, CTREConfigs.CanivoreCANbus);
        m_TurretMotor.getConfigurator().apply(Robot.ctreConfigs.LeftTurretConfig);
        m_TurretMotor.setNeutralMode(NeutralModeValue.Coast);

        e_TurretEncoder = new CANcoder(RobotConstants.FrontLeftTurret.Turret_Rotation_Encoder, CTREConfigs.CanivoreCANbus);
        e_TurretEncoder.getConfigurator().apply(Robot.ctreConfigs.LeftTurretCancoderConfig);
        e_TurretEncoder.setPosition(e_TurretEncoder.getAbsolutePosition().getValueAsDouble());

        TurretPIDController = new PIDController(kP, kI, kD);

        /*Limelight*/
        LeftLimelight = new LimelightAssistant("limelight-llt", VecBuilder.fill(0,0,0), false);
        RightLimelight = new LimelightAssistant("limelight-lrt", VecBuilder.fill(0,0,0), false);


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
                output = MathUtil.clamp(TurretPIDController.calculate(e_TurretEncoder.getPosition().getValueAsDouble(), setPoint), -maxSpeed, maxSpeed);
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

        SmartDashboard.putNumber("LeftTurret Output", output);
        SmartDashboard.putNumber("LeftTurret setPoint", setPoint);
        SmartDashboard.putNumber("LeftTurret Encoder Pose", e_TurretEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("LeftTurret AbsEncoder Pose", e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("TX LeftTurret", TX);
        SmartDashboard.putBoolean("LeftTurretInRange", e_TurretEncoder.getPosition().getValueAsDouble() > NegativeDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < PositiveDeadStop);
        SmartDashboard.putBoolean("LeftTurretInNegRange", e_TurretEncoder.getPosition().getValueAsDouble() < NegativeDeadStop);
        SmartDashboard.putBoolean("LeftTurretInPosRange", e_TurretEncoder.getPosition().getValueAsDouble() > PositiveDeadStop);


        LeftLimelightTargetBoolean();
        RightLimelightTargetBoolean();
        LimeLightTargetBoolean();

        returnPOS();


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

    public double TyValue(){
        return (RightLimelight.getTY());
    }

    public double returnPOS(){
        return e_TurretEncoder.getPosition().getValueAsDouble();
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


