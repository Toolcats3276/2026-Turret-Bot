package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {


    public static CANBus canBus = new CANBus("rio");
    public static CANBus CanivoreCANbus = new CANBus("Everything Else CAN");

    public Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    
    /* Left Shooter */
    public TalonFXConfiguration LeftshooterLeftConfig = new TalonFXConfiguration();
    public TalonFXConfiguration LeftshooterRightConfig = new TalonFXConfiguration();

    public TalonFXConfiguration LeftTurretConfig = new TalonFXConfiguration();

    public CANcoderConfiguration LeftTurretCancoderConfig = new CANcoderConfiguration();
    /* Right Shooter */
    public TalonFXConfiguration RightshooterLeftConfig = new TalonFXConfiguration();
    public TalonFXConfiguration RightshooterRightConfig = new TalonFXConfiguration();

    public TalonFXConfiguration RightTurretConfig = new TalonFXConfiguration();

    public CANcoderConfiguration RightTurretCancoderConfig = new CANcoderConfiguration();
    /* Infeed */
    public TalonFXConfiguration InfeedConfig = new TalonFXConfiguration();

    public TalonFXConfiguration InfeedPivotLeftConfig = new TalonFXConfiguration();
    public TalonFXConfiguration InfeedPivotRightConfig = new TalonFXConfiguration();

    public CANcoderConfiguration InfeedCancoderConfig = new CANcoderConfiguration();
    /* Indexer */
    public TalonFXConfiguration IndexerLeftConfig = new TalonFXConfiguration();
    public TalonFXConfiguration IndexerRightConfig = new TalonFXConfiguration();
    /* Feeder */
    public TalonFXConfiguration FeederConfig = new TalonFXConfiguration();

    public CTREConfigs(){
        gyroConfig.MountPose.MountPoseYaw = 90.31501007080078;
        gyroConfig.MountPose.MountPosePitch = 0.724636971950531;
        gyroConfig.MountPose.MountPoseRoll = -0.6154836416244507;
        // gyroConfig.GyroTrim.GyroScalarZ = -180;

        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /* Right Shooter */
        RightTurretCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        RightTurretCancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        RightTurretCancoderConfig.MagnetSensor.MagnetOffset = 0.02490234375;

        RightshooterLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        RightshooterRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        RightshooterLeftConfig.Slot0.kP = 0.175;
        RightshooterLeftConfig.Slot0.kI = 0;
        RightshooterLeftConfig.Slot0.kD = 0;
        RightshooterLeftConfig.Slot0.kS = 0.225;
        RightshooterLeftConfig.Slot0.kV = 0.0955;


        /* Left Shooter */
        LeftTurretCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        LeftTurretCancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        LeftTurretCancoderConfig.MagnetSensor.MagnetOffset = 0.425048828125;

        LeftshooterLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        LeftshooterRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        LeftshooterLeftConfig.Slot0.kP = 0.25;
        LeftshooterLeftConfig.Slot0.kI = 0;
        LeftshooterLeftConfig.Slot0.kD = 0.01;
        LeftshooterLeftConfig.Slot0.kS = 0.25;
        LeftshooterLeftConfig.Slot0.kV = 0.1;

        /* Infeed */
        InfeedCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        InfeedCancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        InfeedCancoderConfig.MagnetSensor.MagnetOffset = -0.353271484375;

        InfeedPivotLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        InfeedPivotRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Indexer */
        IndexerLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        IndexerRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;



        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    }
}