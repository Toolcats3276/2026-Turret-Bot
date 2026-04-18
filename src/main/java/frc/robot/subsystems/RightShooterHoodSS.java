package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.LeftShooterSS.LeftShooterSetpoints;

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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static frc.robot.Constants.RobotConstants.FrontRightTurret.FUEL_EXIT_ANGLE_OFFSET;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.PITCH_HOME_ANGLE;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.PITCH_LIMIT_FORWARD;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.PITCH_LIMIT_REVERSE;



public class RightShooterHoodSS extends SubsystemBase{

    // private Servo s_LinearActuator;
    private TalonFX m_HoodMotor = new TalonFX(RobotConstants.FrontRightTurret.Hood_Rotation_Motor, CTREConfigs.CanivoreCANbus);
    private CANcoder e_HoodEncoder = new CANcoder(RobotConstants.FrontRightTurret.Hood_Encoder, CTREConfigs.CanivoreCANbus);

    private PositionVoltage HoodPositionVoltage;
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    public static final Angle HOOD_POSITION_TOLERANCE = Degrees.of(.5);
    private double targetRotations;


    private Distance shootangle;
    public static final double HOOD_MOTOR_TO_ENCODER_RATIO = 24.0/18.0;
    public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = 120.0/18.0;
    public static final double YAW_MAGNETIC_OFFSET = 0.708;

    private final double kP = 85;
    private final double kS = 0.44921875;
    private final double kV = 7;
    private final double kD = 0.57;
    
    public RightShooterHoodSS() {
        /*Hood General Rotation*/
        TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(HOOD_MOTOR_TO_ENCODER_RATIO)
            .withFusedCANcoder(e_HoodEncoder)
            .withSensorToMechanismRatio(HOOD_SENSOR_TO_MECHANISM_RATIO))
            .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(kP).withKS(kS).withKV(kV).withKD(kD)));
        m_HoodMotor.getConfigurator().apply(yawTalonConfig); 

        CANcoderConfiguration yawCanCoderConfig = new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs().withMagnetOffset(YAW_MAGNETIC_OFFSET)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1));
        e_HoodEncoder.getConfigurator().apply(yawCanCoderConfig);
        
        // s_LinearActuator = new Servo(6);
        // s_LinearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        HoodPositionVoltage = new PositionVoltage(0).withEnableFOC(true);
        hoodPosition = m_HoodMotor.getPosition();
        hoodVelocity = m_HoodMotor.getVelocity();
    }

     public enum Mode{
        // LinearActuator,
        // stop
    }

    // Mode HoodMode = Mode.stop;
    
    @Override

    public void periodic() {

        // switch(HoodMode) {

            // case LinearActuator:{
            //     s_LinearActuator.set(shootangle.in(Millimeter));
            //     break;
            // }

            // case stop:{
            //   s_LinearActuator.set(0);
            //   break;
            // }

        // }

        // SmartDashboard.putNumber("Right Linear Acutator Angle", s_LinearActuator.get());
        // SmartDashboard.putNumber("Left Linear Actuator Setpoint", shootangle.in(Millimeter));
        SmartDashboard.putNumber("Right Hood Angle", getPitch().in(Degrees));
        SmartDashboard.putNumber("Right Hood Rotations", getPitch().in(Rotations));
        SmartDashboard.putNumber("Right Hood Velocity", m_HoodMotor.getVelocity().getValueAsDouble());

    }

    // public void Stop(){
    //     HoodMode = Mode.stop;
    // }

    // public void LinearActuator(Distance shootangle){
    //     this.shootangle = shootangle;
    //     HoodMode = Mode.LinearActuator;
    // }

    // public Distance LinearActuatorSetPoint(){
    //     return shootangle;
    // }

    // public static record RightShooterConversion(
    //     Angle outputAngle) {
        
    //     public RightShooterConversion interpolate(RightShooterConversion endValue, double t) {
    //       RightShooterConversion result = new RightShooterConversion(
    //         Degrees.of(
    //               MathUtil.interpolate(
    //                   outputAngle.in(Degrees),
    //                     endValue.outputAngle.in(Degrees),
    //                     t))
        
    //         );
    //       return result;
    //     }
    // }

    public Angle getFuelPitch(Angle shooterPitch) {
        // The hood moving up (positive) lowers the exit angle, so the angle has to be subtracted.
        // Turret 0 shoots fuel at FUEL_EXIT_ANGLE_OFFSET
        return FUEL_EXIT_ANGLE_OFFSET.minus(shooterPitch);
    }  

    public void stowPitch() {
        setPitchAngle(PITCH_HOME_ANGLE);
    }

    public void setPitchAngle(Angle targetPitch) {
        m_HoodMotor.setControl(HoodPositionVoltage.withPosition(targetPitch));
    }

    public Angle getPitch() {
        BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity);
        return BaseStatusSignal.getLatencyCompensatedValue(hoodPosition, hoodVelocity);
    }

    public boolean isPitchAtSetpoint() {
      BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity);
      Angle currentPitch = BaseStatusSignal.getLatencyCompensatedValue(hoodPosition, hoodVelocity);
      return MathUtil.isNear(HoodPositionVoltage.Position, currentPitch.in(Rotations), HOOD_POSITION_TOLERANCE.in(Rotations));
    }
}




