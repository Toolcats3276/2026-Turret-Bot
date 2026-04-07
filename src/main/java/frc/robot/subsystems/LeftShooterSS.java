package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.CTREConfigs;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.vision.LimelightAssistant;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.ROBOT_TO_LEFT_SHOOTER;


/* SEE WristSS FOR EXPLANATIONS */

public class LeftShooterSS extends SubsystemBase {

    private TalonFX m_shooterLeftMotor = new TalonFX(RobotConstants.FrontLeftTurret.Shoot_Motor, CTREConfigs.CanivoreCANbus);

    private final double kP = .35;
    private final double kS = 0.225;
    private final double kV = .126;
    // private TalonFX m_shooterRightMotor;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<AngularAcceleration> flywheelAcceleration;
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
    public static final AngularVelocity FLYWHEEL_VELOCITY_TOLERANCE = RotationsPerSecond.of(1);

    private AngularVelocity speed;

    public LeftShooterSS(){
        TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(kP).withKS(kS).withKV(kV)));
        
        m_shooterLeftMotor.getConfigurator().apply(yawTalonConfig);
            

            // m_shooterRightMotor = new TalonFX(RobotConstants.FrontRightTurret.Shoot_Motor_Right_Motor, CTREConfigs.CanivoreCANbus);
            // m_shooterRightMotor.getConfigurator().apply(Robot.ctreConfigs.RightshooterRightConfig);
            // m_shooterRightMotor.setNeutralMode(NeutralModeValue.Coast);
            // m_shooterRightMotor.setControl(new StrictFollower(m_shooterLeftMotor.getDeviceID()));

        flywheelVelocity = m_shooterLeftMotor.getVelocity();
        flywheelAcceleration = m_shooterLeftMotor.getAcceleration();
    }


    public enum Mode{
        Stop,
        SetSpeed,
    }

    Mode ShooterMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(ShooterMode) {

            case Stop:{
                m_shooterLeftMotor.set(0);
                break;
            }

            case SetSpeed:{
                // m_shooterLeftMotor.set(speed.in(RotationsPerSecond));
                m_shooterLeftMotor.setControl(flywheelVelocityRequest.withVelocity(speed.in(RotationsPerSecond)));
            }

        }

        SmartDashboard.putNumber("LeftShooterCurrentSpeed", m_shooterLeftMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("RightRightShooterCurrentSpeed", m_shooterRightMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("LeftFlyWheelAtSpeed", isFlywheelAtSpeed());

    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    public static record LeftShooterSetpoints(
        Angle shotAngle,
        AngularVelocity shotVelocity) {
        
    public LeftShooterSetpoints interpolate(LeftShooterSetpoints endValue, double t) {
      LeftShooterSetpoints result = new LeftShooterSetpoints(
        Degrees.of(MathUtil.interpolate(shotAngle.in(Degrees), endValue.shotAngle.in(Degrees), t)),
        RotationsPerSecond.of(
              MathUtil.interpolate(
                  shotVelocity.in(RotationsPerSecond),
                    endValue.shotVelocity.in(RotationsPerSecond),
                    t))
  
        );
      return result;
    }
    }

    public static Translation2d getShooterTranslation(Pose2d robotPose) {
        return robotPose.getTranslation().plus(ROBOT_TO_LEFT_SHOOTER.rotateBy(robotPose.getRotation()));
    }

        /** Returns whether flywheel speed is within configured tolerance of the active request. */
    
    public boolean isFlywheelAtSpeed() {
        BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAcceleration);
        AngularVelocity currentSpeed = BaseStatusSignal.getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
        return MathUtil.isNear(
            flywheelVelocityRequest.Velocity,
            currentSpeed.in(RotationsPerSecond),
            FLYWHEEL_VELOCITY_TOLERANCE.in(RotationsPerSecond));
    }
}