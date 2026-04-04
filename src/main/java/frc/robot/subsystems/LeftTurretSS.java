package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.CTREConfigs;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.vision.LimelightAssistant;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_RANGE_FORWARD;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_RANGE_REVERSE;


public class LeftTurretSS extends SubsystemBase{
    /*General Turret Rotation*/
    private TalonFX m_TurretMotor = new TalonFX(RobotConstants.FrontLeftTurret.Turret_Rotation_Motor, CTREConfigs.CanivoreCANbus);
    private CANcoder e_TurretEncoder = new CANcoder(RobotConstants.FrontLeftTurret.Turret_Rotation_Encoder, CTREConfigs.CanivoreCANbus);

    private PIDController TurretPIDController;

    private final double kP = 75;
    private final double kS = 0.47;
    private final double kV = 10;
    private final double kD = 4.75;

    private PositionVoltage leftTurretPositionVoltage;

    public static final MotionMagicConfigs YAW_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(9.5)
        .withMotionMagicCruiseVelocity(10.0);
    public static final double YAW_MOTOR_TO_SENSOR_RATIO = 1;
    public static final double YAW_SENSOR_TO_Turret_RATIO = 192.0/18.0;
    public static final double YAW_MAGNETIC_OFFSET = 0.422607421875;

    private final StatusSignal<Angle> yawPosition;
    private final StatusSignal<AngularVelocity> yawVelocity;
    public static final Angle YAW_POSITION_TOLERANCE = Degrees.of(2.5);
    private double targetRotations;


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
        TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs().withRotorToSensorRatio(YAW_MOTOR_TO_SENSOR_RATIO)
                    .withFusedCANcoder(e_TurretEncoder)
                    .withSensorToMechanismRatio(YAW_SENSOR_TO_Turret_RATIO))
            .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(kP).withKS(kS).withKV(kV).withKD(kD)))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(YAW_LIMIT_FORWARD)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(YAW_LIMIT_REVERSE));
        
        m_TurretMotor.getConfigurator().apply(yawTalonConfig);
        

        CANcoderConfiguration yawCanCoderConfig = new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs().withMagnetOffset(YAW_MAGNETIC_OFFSET)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(.5));
        // e_TurretEncoder.setPosition(e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        e_TurretEncoder.getConfigurator().apply(yawCanCoderConfig);

        /*Limelight*/
        LeftLimelight = new LimelightAssistant("limelight-llt", VecBuilder.fill(0,0,0), false);
        RightLimelight = new LimelightAssistant("limelight-lrt", VecBuilder.fill(0,0,0), false);


        LLRotationPidController = new PIDController(LLkP, LLkI, LLkD);

        leftTurretPositionVoltage = new PositionVoltage(0).withEnableFOC(true);

        yawPosition = m_TurretMotor.getPosition();
        yawVelocity = m_TurretMotor.getVelocity();

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
        SmartDashboard.putNumber("Left Turret Yaw", getYaw().in(Rotations));
        SmartDashboard.putBoolean("LeftYawCorrect", isYawAtSetpoint());




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

    public void setYawAngle(Angle targetYaw) {
        // Wrap the input to match the range of the turret. The input is probably in the range of (-0.5, 0.5], but the
        // turret range is more like [-0.75, 0.25].
        targetRotations = -MathUtil
            .inputModulus(targetYaw.in(Rotations), YAW_RANGE_REVERSE.in(Rotations), YAW_RANGE_FORWARD.in(Rotations));
        Angle currentYaw = getYaw();
        m_TurretMotor.setControl(leftTurretPositionVoltage.withPosition(targetRotations));
    }

    public Angle getYaw() {
        BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
        return BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    }

    public boolean isYawAtSetpoint() {
      BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
      Angle currentYaw = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
      return MathUtil.isNear(leftTurretPositionVoltage.Position, currentYaw.in(Rotations), YAW_POSITION_TOLERANCE.in(Rotations));
    }

}


