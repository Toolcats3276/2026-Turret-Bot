package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.vision.LimelightAssistant;
import frc.robot.Constants.Swerve;
import frc.robot.Robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSS extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;
    
    public static PIDController LLTranslationFR;
    public static PIDController LLStrafeFR;
    public static PIDController LLTranslationFL;
    public static PIDController LLStrafeFL;

    public static LimelightAssistant LLAssistantFR;
    public static LimelightAssistant LLAssistantFL;


    public static SwerveDrivePoseEstimator m_poseEstimator;

    public Field2d LLPose;
    public Field2d BotPose;
    public boolean scoreLeft;


    @SuppressWarnings("unused")
    private NeutralModeValue driveNeutralMode;
    
    
        public SwerveSS() {
    
            driveNeutralMode = NeutralModeValue.Brake;
    
            gyro = new Pigeon2(Swerve.pigeonID);
            gyro.getConfigurator().apply(new Pigeon2Configuration());
            gyro.setYaw(0);
    
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Swerve.Mod0.constants),
                new SwerveModule(1, Swerve.Mod1.constants),
                new SwerveModule(2, Swerve.Mod2.constants),
                new SwerveModule(3, Swerve.Mod3.constants)
            };
    
            swerveOdometry = new SwerveDriveOdometry(Swerve.swerveKinematics, getGyroYaw(), getModulePositions());


            LLTranslationFR = new PIDController(.09, 0, 0.02);
            LLStrafeFR = new PIDController(0.01, 0, 0.0005);
            LLTranslationFL = new PIDController(.037, 0, 0.03);
            LLStrafeFL = new PIDController(0.007, 0, 0.0009);

            LLAssistantFR = new LimelightAssistant("limelight-fr", VecBuilder.fill(0,0,0), false);
            LLAssistantFL = new LimelightAssistant("limelight-fl", VecBuilder.fill(0,0,0), false);
            
    
                
        }
    
        public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
            SwerveModuleState[] swerveModuleStates =
                Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            translation.getX(), 
                            translation.getY(), 
                            rotation, 
                            getHeading()
                        )
                        : new ChassisSpeeds(
                            translation.getX(), 
                            translation.getY(), 
                            rotation)
                        );
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.maxSpeed);
    
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }  
        
        
        public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
            ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
            SwerveModuleState[] targetState = Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
            // SwerveDriveKinematics.desaturateWheelSpeeds(targetState, Swerve.maxSpeed);
    
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(targetState[mod.moduleNumber], false);
            }
    
          }
        // public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        //     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        
        //     SwerveModuleState[] targetStates = Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        //     setModuleStates(targetStates);
        //   }
    
        /* Used by SwerveControllerCommand in Auto */
        public void setModuleStates(SwerveModuleState[] desiredStates) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);
            
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            }
        }
    
        public SwerveModuleState[] getModuleStates(){
            SwerveModuleState[] states = new SwerveModuleState[4];
            for(SwerveModule mod : mSwerveMods){
                states[mod.moduleNumber] = mod.getState();
            }
            return states;
        }
    
        public SwerveModulePosition[] getModulePositions(){
            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            for(SwerveModule mod : mSwerveMods){
                positions[mod.moduleNumber] = mod.getPosition();
            }
            return positions;
        }
    
        public ChassisSpeeds getRobotSpeed(){
            return Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        }
    
        public Pose2d getPose() {
            return swerveOdometry.getPoseMeters();
        }
    
        public Pose2d getPoseEstimate(){
            return m_poseEstimator.getEstimatedPosition();
        }
    
        public void setPose(Pose2d pose) {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
    
        public void resetPoseEstimate(Pose2d pose){
            m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
    
        public Rotation2d getHeading(){
            return getPose().getRotation();
        }
    
        public void setHeading(Rotation2d heading){
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        }
    
        public void zeroHeading(){
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }
    
        public Rotation2d getGyroYaw() {
            return Rotation2d.fromDegrees(gyro.getYaw().getValue().in(Degrees));
        }
    
        public void resetModulesToAbsolute(){
            for(SwerveModule mod : mSwerveMods){
                mod.resetToAbsolute();
            }
        }
    
    
        public void setNeutralMode(NeutralModeValue driveNeutralMode){
            this.driveNeutralMode = driveNeutralMode;
            TalonFXConfiguration swerveDriveFXConfig = Robot.ctreConfigs.swerveDriveFXConfig;
            swerveDriveFXConfig.MotorOutput.NeutralMode = driveNeutralMode;

            mSwerveMods[0].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
            mSwerveMods[1].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
            mSwerveMods[2].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
            mSwerveMods[3].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
        }


    @SuppressWarnings("removal")
    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        


        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }



    }
}