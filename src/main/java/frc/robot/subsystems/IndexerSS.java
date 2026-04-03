package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

/* SEE WristSS FOR EXPLANATIONS */

public class IndexerSS extends SubsystemBase {

    private TalonFX m_Motor = new TalonFX(RobotConstants.Indexer.Indexer_Motor_Left, CTREConfigs.CanivoreCANbus);

    private final double kP = .45;
    private final double kS = 0.46;
    private final double kV = .13;
    // private TalonFX m_shooterRightMotor;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<AngularAcceleration> flywheelAcceleration;
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
    public static final AngularVelocity FLYWHEEL_VELOCITY_TOLERANCE = RotationsPerSecond.of(1.5);

    private AngularVelocity speed;

    public IndexerSS(){
        TalonFXConfiguration yawTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
            // .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(kP).withKS(kS).withKV(kV)))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(70).withSupplyCurrentLimit(25).withSupplyCurrentLowerLimit(16).withSupplyCurrentLowerTime(1));
        
        m_Motor.getConfigurator().apply(yawTalonConfig);

        flywheelVelocity = m_Motor.getVelocity();
        flywheelAcceleration = m_Motor.getAcceleration();
    }


    public enum Mode{
        Stop,
        SetSpeed,
    }

    Mode IndexerMode = Mode.Stop;
    
    @Override

    public void periodic() {

        switch(IndexerMode) {

            case Stop:{
                m_Motor.set(0);
                break;
            }

            case SetSpeed:{
                // m_Motor.setControl(flywheelVelocityRequest.withVelocity(speed.in(RotationsPerSecond)));
                m_Motor.set(speed.in(RotationsPerSecond));
            }
        }

        // SmartDashboard.putNumber("IndexerSetSpeed", speed.in(RotationsPerSecond));
        SmartDashboard.putNumber("IndexerCurrentSpeed", m_Motor.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        IndexerMode = Mode.Stop;
    }
    
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        IndexerMode = Mode.SetSpeed;
    } 
}