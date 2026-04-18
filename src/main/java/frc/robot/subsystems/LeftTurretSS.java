package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
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
import com.ctre.phoenix6.controls.PositionVoltage;
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

import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_RANGE_FORWARD;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.YAW_RANGE_REVERSE;


public class LeftTurretSS extends SubsystemBase{
    /*General Turret Rotation*/
    private TalonFX m_TurretMotor = new TalonFX(RobotConstants.FrontLeftTurret.Turret_Rotation_Motor, CTREConfigs.CanivoreCANbus);
    private CANcoder e_TurretEncoder = new CANcoder(RobotConstants.FrontLeftTurret.Turret_Rotation_Encoder, CTREConfigs.CanivoreCANbus);

    private PositionVoltage TurretPositionVoltage;
    private final StatusSignal<Angle> yawPosition;
    private final StatusSignal<AngularVelocity> yawVelocity;
    public static final Angle YAW_POSITION_TOLERANCE = Degrees.of(1.5);


    private final double kP = 75;
    private final double kS = 0.47;
    private final double kV = 10;
    private final double kD = 4.75;

    public static final double YAW_MOTOR_TO_SENSOR_RATIO = 1;
    public static final double YAW_SENSOR_TO_Turret_RATIO = 192.0/18.0;
    public static final double YAW_MAGNETIC_OFFSET = 0.400390625;

    private double output;
    private double targetRotations;
    
    public LeftTurretSS() {
        /*Turret General Rotation*/
        TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(YAW_MOTOR_TO_SENSOR_RATIO)
            .withFusedCANcoder(e_TurretEncoder)
            .withSensorToMechanismRatio(YAW_SENSOR_TO_Turret_RATIO))
            .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(kP).withKS(kS).withKV(kV).withKD(kD)))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(YAW_LIMIT_FORWARD)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(YAW_LIMIT_REVERSE));
        m_TurretMotor.getConfigurator().apply(yawTalonConfig);
        

        CANcoderConfiguration yawCanCoderConfig = new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs().withMagnetOffset(YAW_MAGNETIC_OFFSET)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(.5));
        e_TurretEncoder.getConfigurator().apply(yawCanCoderConfig);

        TurretPositionVoltage = new PositionVoltage(0).withEnableFOC(true);
        yawPosition = m_TurretMotor.getPosition();
        yawVelocity = m_TurretMotor.getVelocity();
        
    }
    
    @Override

    public void periodic() {

        SmartDashboard.putNumber("LeftTurret Output", output);
        SmartDashboard.putNumber("LeftTurret Encoder Pose", e_TurretEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("LeftTurret Rotation Encoder Pose", e_TurretEncoder.getPosition().getValue().in(Rotations));
        SmartDashboard.putNumber("LeftTurret AbsEncoder Pose", e_TurretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Left Turret Yaw", getYaw().in(Rotations));
        SmartDashboard.putBoolean("LeftYawCorrect", isYawAtSetpoint());

    }

    public void stowYaw(){
        setYawAngle(Rotations.of(0));
    }

    public void setYawAngle(Angle targetYaw) {
        // Wrap the input to match the range of the turret. The input is probably in the range of (-0.5, 0.5], but the
        // turret range is more like [-0.75, 0.25].
        targetRotations = -MathUtil
            .inputModulus(targetYaw.in(Rotations), YAW_RANGE_REVERSE.in(Rotations), YAW_RANGE_FORWARD.in(Rotations));
        m_TurretMotor.setControl(TurretPositionVoltage.withPosition(targetRotations));
    }

    public Angle getYaw() {
        BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
        return BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    }

    public boolean isYawAtSetpoint() {
      BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
      Angle currentYaw = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
      return MathUtil.isNear(TurretPositionVoltage.Position, currentYaw.in(Rotations), YAW_POSITION_TOLERANCE.in(Rotations));
    }
}   


