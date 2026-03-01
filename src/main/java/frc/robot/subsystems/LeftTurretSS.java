package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
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
    private Servo s_LinearActuator;

    private PIDController TurretPIDController;

    private final double kP = 0.25;
    private final double kI = 0;
    private final double kD = 0;

    private double output;
    private double shootangle;
    private double setPoint;
    private double maxSpeed;
    private double ManualVal;
    private double TX;
    private double shotPower;
    private double shooterAngle;

    private double NegativeDeadStop = -2.667;
    private double PositiveDeadStop = 2.667;
    
    /*Limelight*/

    private final LimelightAssistant LeftLimelight;
    private final LimelightAssistant RightLimelight;
    private LimelightAssistant center_Limmelight;


    private final PIDController LLRotationPidController;
        private final double LLkP = 0.008;
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
        center_Limmelight = new LimelightAssistant("limelight-ty", VecBuilder.fill(0,0,0), false);


        LLRotationPidController = new PIDController(LLkP, LLkI, LLkD);

        s_LinearActuator = new Servo(2);
        s_LinearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

     public enum Mode{
        Stop,
        PID,
        AutoAim,
        Manual,
        LinearActuator,
        ShooterAutoAim
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

            case LinearActuator:{
                s_LinearActuator.set(shootangle);
                break;
            }

            case ShooterAutoAim:{
                shotPower = ((((7.49052) * Math.pow(10, -7)) * (Math.pow(center_Limmelight.getTY(), 4))) - ((0.0000363256)*(Math.pow(center_Limmelight.getTY(), 3)))+((0.000564661)*(Math.pow(center_Limmelight.getTY(), 2)))+((0.00128608)*(center_Limmelight.getTY())) + 0.522387);
                break;            
            }

            
        }

        SmartDashboard.putNumber("LeftTurret Output", output);
        SmartDashboard.putNumber("LeftTurret setPoint", setPoint);
        SmartDashboard.putNumber("LeftTurret Encoder Pose", e_TurretEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("LeftTurret AbsEncoder Pose", e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("LeftTurret_P", kP);
        SmartDashboard.putNumber("TX LeftTurret", TX);
        SmartDashboard.putBoolean("LeftTurretInRange", e_TurretEncoder.getPosition().getValueAsDouble() > NegativeDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < PositiveDeadStop);
        SmartDashboard.putBoolean("LeftTurretInNegRange", e_TurretEncoder.getPosition().getValueAsDouble() < NegativeDeadStop);
        SmartDashboard.putBoolean("LeftTurretInPosRange", e_TurretEncoder.getPosition().getValueAsDouble() > PositiveDeadStop);
        SmartDashboard.putNumber("Left Linear Acutator Angle", s_LinearActuator.get());
        SmartDashboard.putNumber("Left Linear Actuator Setpoint", shootangle);

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

    public void LinearActuator(double shootangle){
        this.shootangle = shootangle;
        TurretMode = Mode.LinearActuator;
    }

    public double LinearActuatorSetPoint(){
        return shootangle;
    }

    public double TxValue(){
        TX = LeftLimelight.getTX() + RightLimelight.getTX();
        return TX;
    }

    public double TyValue(){
        return (LeftLimelight.getTY() + RightLimelight.getTY())/2;
    }
    
    public double ShootPower(){   
        TurretMode = Mode.ShooterAutoAim;
        return shotPower;
    }

    public double returnPOS(){
        return e_TurretEncoder.getPosition().getValueAsDouble();
    }

}


