package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.vision.LimelightAssistant;

import static frc.robot.Constants.RobotConstants.FrontRightTurret.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.YAW_RANGE_FORWARD;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.YAW_RANGE_REVERSE;


public class RightTurretSS extends SubsystemBase{
    /*General Turret Rotation*/
    private TalonFX m_TurretMotor = new TalonFX(RobotConstants.FrontRightTurret.Turret_Rotation_Motor, CTREConfigs.CanivoreCANbus);
    private CANcoder e_TurretEncoder = new CANcoder(RobotConstants.FrontRightTurret.Turret_Rotation_Encoder, CTREConfigs.CanivoreCANbus);

    private PIDController TurretPIDController;

    private MotionMagicVoltage rightTurretMotionMagicVoltage;
    private final StatusSignal<Angle> yawPosition;
    private final StatusSignal<AngularVelocity> yawVelocity;
    public static final Angle YAW_POSITION_TOLERANCE = Degrees.of(2.5);


    private final double kP = 12;
    private final double kS = 0.42;
    private final double kV = 1;

    public static final MotionMagicConfigs YAW_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(12.0)
        .withMotionMagicCruiseVelocity(10.0);
    public static final double YAW_MOTOR_TO_SENSOR_RATIO = 1;
    public static final double YAW_SENSOR_TO_Turret_RATIO = 192/18;
    public static final double YAW_MAGNETIC_OFFSET = 0.02490234375;
    public static final Angle YAW_ENCODER_DISCONTINUITY_POINT = YAW_LIMIT_REVERSE.plus(YAW_LIMIT_FORWARD)
        .div(2.0)
        .plus(Rotations.of(0.5));

    private double output;
    private double shootangle;
    private double setPoint;
    private double maxSpeed;
    private double ManualVal;
    private double TX;
    private double shotPower;
    private double shooterAngle;
    private double targetRotations;

    private double s_speed;
    private double s_length;
    private double s_setPoint;
    private double s_currentpos;

    private double NegativeDeadStop = -3.2;
    private double PositiveDeadStop = 3.2;
    
    /*Limelight*/

    public final LimelightAssistant LeftLimelight;
    public final LimelightAssistant RightLimelight;

    private final PIDController LLRotationPidController;
        private final double LLkP = 0.004;
        private final double LLkI = 0;
        private final double LLkD = 0;
    
    public RightTurretSS() {
        /*Turret General Rotation*/
        TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs().withRotorToSensorRatio(YAW_MOTOR_TO_SENSOR_RATIO)
                    .withFusedCANcoder(e_TurretEncoder)
                    .withSensorToMechanismRatio(YAW_SENSOR_TO_Turret_RATIO))
            .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(kP).withKS(kS).withKV(kV)))
            .withMotionMagic(YAW_MOTION_MAGIC_CONFIGS)
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(YAW_LIMIT_FORWARD)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(YAW_LIMIT_REVERSE));
        
        m_TurretMotor.getConfigurator().apply(yawTalonConfig);
        

        CANcoderConfiguration yawCanCoderConfig = new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs().withMagnetOffset(YAW_MAGNETIC_OFFSET)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(YAW_ENCODER_DISCONTINUITY_POINT));
        e_TurretEncoder.setPosition(e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        e_TurretEncoder.getConfigurator().apply(yawCanCoderConfig);

        /*Limelight*/
        LeftLimelight = new LimelightAssistant("limelight-rlt", VecBuilder.fill(0,0,0), false);
        RightLimelight = new LimelightAssistant("limelight-rrt", VecBuilder.fill(0,0,0), false);

        LLRotationPidController = new PIDController(LLkP, LLkI, LLkD);

        rightTurretMotionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(true);
        yawPosition = e_TurretEncoder.getPosition();
        yawVelocity = m_TurretMotor.getVelocity();
        
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

        SmartDashboard.putNumber("RightTurret Output", output);
        SmartDashboard.putNumber("RightTurret setPoint", setPoint);
        SmartDashboard.putNumber("RightTurret Encoder Pose", e_TurretEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RightTurret Rotation Encoder Pose", e_TurretEncoder.getPosition().getValue().in(Rotations));
        SmartDashboard.putNumber("RightTurret AbsEncoder Pose", e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("TX RightTurret", TX);
        SmartDashboard.putBoolean("RightTurretInRange", e_TurretEncoder.getPosition().getValueAsDouble() > NegativeDeadStop && e_TurretEncoder.getPosition().getValueAsDouble() < PositiveDeadStop);
        SmartDashboard.putBoolean("RightTurretInNegRange", e_TurretEncoder.getPosition().getValueAsDouble() < NegativeDeadStop);
        SmartDashboard.putBoolean("RightTurretInPosRange", e_TurretEncoder.getPosition().getValueAsDouble() > PositiveDeadStop);
        SmartDashboard.putNumber("Turret Yaw", getYaw().in(Rotations));
        SmartDashboard.putNumber("Right Turret Target", targetRotations);

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

    public void setYawAngle(Angle targetYaw) {
        // Wrap the input to match the range of the turret. The input is probably in the range of (-0.5, 0.5], but the
        // turret range is more like [-0.75, 0.25].
        targetRotations = MathUtil
            .inputModulus(targetYaw.in(Rotations), YAW_RANGE_REVERSE.in(Rotations), YAW_RANGE_FORWARD.in(Rotations));
        Angle currentYaw = getYaw();
        double ffVolts = 0.0;
        if (currentYaw.gt(Rotations.of(.5))) {
          ffVolts = .5;
        } else if (currentYaw.lt(Rotations.of(-0.5))) {
          ffVolts = -1;
        }
        m_TurretMotor.setControl(rightTurretMotionMagicVoltage.withPosition(targetRotations).withFeedForward(Volts.of(ffVolts)));
    }

    public Angle getYaw() {
        BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
        return BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    }

    public boolean isYawAtSetpoint() {
      BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
      Angle currentYaw = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
      return MathUtil.isNear(rightTurretMotionMagicVoltage.Position, currentYaw.in(Rotations), YAW_POSITION_TOLERANCE.in(Rotations));
    }


}   


