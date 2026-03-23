package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import static frc.robot.Constants.RobotConstants.FrontRightTurret.ROBOT_TO_Right_SHOOTER;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.FUEL_EXIT_ANGLE_OFFSET;


/* SEE WristSS FOR EXPLANATIONS */

public class RightShooterSS extends SubsystemBase {

    private TalonFX m_shooterLeftMotor;
    private TalonFX m_shooterRightMotor;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<AngularAcceleration> flywheelAcceleration;
    private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    public static final AngularVelocity FLYWHEEL_VELOCITY_TOLERANCE = RotationsPerSecond.of(1.5);

    private AngularVelocity speed;

    public RightShooterSS(){
            m_shooterLeftMotor = new TalonFX(RobotConstants.FrontRightTurret.Shoot_Motor_Left_Motor, CTREConfigs.CanivoreCANbus);
            m_shooterLeftMotor.getConfigurator().apply(Robot.ctreConfigs.RightshooterLeftConfig);
            m_shooterLeftMotor.setNeutralMode(NeutralModeValue.Coast);
            

            m_shooterRightMotor = new TalonFX(RobotConstants.FrontRightTurret.Shoot_Motor_Right_Motor, CTREConfigs.CanivoreCANbus);
            m_shooterRightMotor.getConfigurator().apply(Robot.ctreConfigs.RightshooterRightConfig);
            m_shooterRightMotor.setNeutralMode(NeutralModeValue.Coast);
            m_shooterRightMotor.setControl(new StrictFollower(m_shooterLeftMotor.getDeviceID()));

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

        SmartDashboard.putNumber("RightLeftShooterCurrentSpeed", m_shooterLeftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("RightRightShooterCurrentSpeed", m_shooterRightMotor.getVelocity().getValueAsDouble());

    }

    public void Stop(){
        ShooterMode = Mode.Stop;
    }
    
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        ShooterMode = Mode.SetSpeed;
    }

    public static record RightShooterSetpoints(
        Distance shotAngle,
        AngularVelocity shotVelocity) {
        
    public RightShooterSetpoints interpolate(RightShooterSetpoints endValue, double t) {
      RightShooterSetpoints result = new RightShooterSetpoints(
        Millimeters.of(MathUtil.interpolate(shotAngle.in(Millimeters), endValue.shotAngle.in(Millimeters), t)),
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
        return robotPose.getTranslation().plus(ROBOT_TO_Right_SHOOTER.rotateBy(robotPose.getRotation()));
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