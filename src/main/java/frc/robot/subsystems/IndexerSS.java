package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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

    private TalonFX m_LeftMotor = new TalonFX(RobotConstants.Indexer.Indexer_Motor_Left, CTREConfigs.CanivoreCANbus);
    private TalonFX m_RightMotor = new TalonFX(RobotConstants.Indexer.Indexer_Motor_Right, CTREConfigs.CanivoreCANbus);

    private final double kP = 12;
    private final double kS = 13;
    private final double kV = .28;
    // private TalonFX m_shooterRightMotor;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<AngularAcceleration> flywheelAcceleration;
    private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

    private AngularVelocity speed;

    public IndexerSS(){
        TalonFXConfiguration LeftTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKS(kS).withKV(kV).withKP(kP))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(150)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(80)
                    .withSupplyCurrentLimitEnable(true));

        TalonFXConfiguration RightTalonConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast));
        m_RightMotor.setControl(new StrictFollower(m_LeftMotor.getDeviceID()));
            // .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(kP).withKS(kS).withKV(kV)))
            // .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(70).withSupplyCurrentLimit(25).withSupplyCurrentLowerLimit(16).withSupplyCurrentLowerTime(1));
        
        m_LeftMotor.getConfigurator().apply(LeftTalonConfig);
        m_RightMotor.getConfigurator().apply(RightTalonConfig);

        flywheelVelocity = m_LeftMotor.getVelocity();
        flywheelAcceleration = m_LeftMotor.getAcceleration();
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
                m_LeftMotor.set(0);
                break;
            }

            case SetSpeed:{
                // m_LeftMotor.setControl(flywheelVelocityRequest.withVelocity(speed.in(RotationsPerSecond)));
                m_LeftMotor.set(speed.in(RotationsPerSecond));
            }
        }

        // SmartDashboard.putNumber("IndexerSetSpeed", speed.in(RotationsPerSecond));
        SmartDashboard.putNumber("IndexerCurrentSpeed", m_LeftMotor.getVelocity().getValueAsDouble());
    }

    public void Stop(){
        IndexerMode = Mode.Stop;
    }
    
    public void setSpeed(AngularVelocity speed){
        this.speed = speed;
        IndexerMode = Mode.SetSpeed;
    } 
}