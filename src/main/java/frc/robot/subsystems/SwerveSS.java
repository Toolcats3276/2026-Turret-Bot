package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.Swerve;
import static frc.robot.Constants.RobotConstants.VisionConstants.APRILTAG_CAMERA_NAMES;
import static frc.robot.Constants.RobotConstants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;
import static frc.robot.Constants.RobotConstants.VisionConstants.APRILTAG_STD_DEVS;
import static frc.robot.Constants.RobotConstants.VisionConstants.TAG_DISTANCE_THRESHOLD;
import static frc.robot.Constants.RobotConstants.VisionConstants.ANGULAR_VELOCITY_THRESHOLD;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import org.ejml.equation.IntegerSequence.For;

import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSS extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro = new Pigeon2(Swerve.pigeonID);
    public RobotConfig config;

    public static SwerveDrivePoseEstimator m_poseEstimator;

    public Field2d LLPose;
    public Field2d BotPose;

    private final Field2d m_field = new Field2d();
    private final Field2d m_LLfield = new Field2d();


    @SuppressWarnings("unused")
    private NeutralModeValue driveNeutralMode;
    
    
        public SwerveSS() {
    
            driveNeutralMode = NeutralModeValue.Brake;

            Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration()
                .withMountPose(new MountPoseConfigs().withMountPosePitch(0).withMountPoseRoll(0).withMountPoseYaw(0));
    
            gyro.getConfigurator().apply(pigeon2Configuration);
            // gyro.setYaw(180);
        
    
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Swerve.Mod0.constants),
                new SwerveModule(1, Swerve.Mod1.constants),
                new SwerveModule(2, Swerve.Mod2.constants),
                new SwerveModule(3, Swerve.Mod3.constants)
            };
    
            swerveOdometry = new SwerveDrivePoseEstimator(Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
            m_poseEstimator = new SwerveDrivePoseEstimator(Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
            // m_poseEstimator = new PoseEstimator<>(Swerve.swerveKinematics, null, VecBuilder.fill(0, 0, 0), VecBuilder.fill(.5,.5,9999999));

            
            try{
                  config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                  // Handle exception as needed
                  e.printStackTrace();
            }

            AutoBuilder.configure(
                this::getPose, 
                this::setPose, 
                this::getRobotSpeed, 
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(3.5, 0, 0.1), // Translation constants //3.5
                    new PIDConstants(1.75, 0, 0)  // Rotation constants P = .1
                ),
                config,
                // () -> false,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, 
                this);
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

        public void Xdrive(boolean isOpenLoop) {
            SwerveModuleState[] swerveModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            };
                
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }  
        
 
        
        
        public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
            // Match the same rotation convention teleop currently uses so PathPlanner
            // commands turn in the same direction as the driver-controlled path.
            ChassisSpeeds correctedSpeeds = new ChassisSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond,
                -robotRelativeSpeeds.omegaRadiansPerSecond
            );
            ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(correctedSpeeds, 0.02);
    
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

        public ChassisSpeeds getCurrentFieldChassisSpeeds() {
            var robotAngle = getHeading();
            var chassisSpeeds = Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
            var fieldSpeeds = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(robotAngle);
            return new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
        }
    
        public Pose2d getPose() {
            return swerveOdometry.getEstimatedPosition();
        }
    
        public Pose2d getPoseEstimate(){
            return m_poseEstimator.getEstimatedPosition();
        }
    
        public void setPose(Pose2d pose) {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
    
        // public void resetPoseEstimate(Pose2d pose){
        //     m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        // }
    
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
            return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
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
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        getPoseEstimate();
        getPose();
        

        m_field.setRobotPose(swerveOdometry.getEstimatedPosition());
        m_LLfield.setRobotPose(getPoseEstimate());
        
        

        SmartDashboard.putData("BotPose", m_field);
        SmartDashboard.putData("LLBotPose", m_LLfield);
        
        SmartDashboard.putNumber("GetHeading", getHeading().getDegrees());
        
        
        LimelightHelpers.PoseEstimate mt1l = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-l");
        LimelightHelpers.PoseEstimate mt1f = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-f");
        LimelightHelpers.PoseEstimate mt1r = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-r");
        
        boolean doRejectUpdater = false;
        boolean doRejectUpdatef = false;
        boolean doRejectUpdatel = false;
        
            if(mt1l.tagCount == 1 && mt1l.rawFiducials.length == 1)
            {
                if(mt1l.rawFiducials[0].ambiguity > .7)
                {
                    doRejectUpdatel = true;
                }
                if(mt1l.rawFiducials[0].distToCamera > 1)
                {
                    doRejectUpdatel = true;
                }
            }
            if(mt1l.tagCount == 0)
            {
                doRejectUpdatel = true;
            }
            
            if(!doRejectUpdatel)
            {
              swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
              swerveOdometry.addVisionMeasurement(
                  mt1l.pose.toPose2d(),
                  mt1l.timestampSeconds);
            }

            if(mt1f.tagCount == 1 && mt1f.rawFiducials.length == 1)
            {
                if(mt1f.rawFiducials[0].ambiguity > .7)
                {
                    doRejectUpdatef = true;
                }
                if(mt1f.rawFiducials[0].distToCamera > 3)
                {
                    doRejectUpdatef = true;
                }
            }
            if(mt1f.tagCount == 0)
            {
                doRejectUpdatef = true;
            }
            
            if(!doRejectUpdatef)
            {
              swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
              swerveOdometry.addVisionMeasurement(
                  mt1f.pose.toPose2d(),
                  mt1f.timestampSeconds);
            }
            
            if(mt1r.tagCount == 1 && mt1r.rawFiducials.length == 1)
            {
                if(mt1r.rawFiducials[0].ambiguity > .7)
                {
                    doRejectUpdater = true;
                }
                if(mt1r.rawFiducials[0].distToCamera > 3)
                {
                    doRejectUpdater = true;
                }
            }
            if(mt1r.tagCount == 0)
            {
                doRejectUpdater = true;
            }
            
            if(!doRejectUpdater)
            {
              swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
              swerveOdometry.addVisionMeasurement(
                  mt1r.pose.toPose2d(),
                  mt1r.timestampSeconds);
            }
            
            

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }



    }
}
