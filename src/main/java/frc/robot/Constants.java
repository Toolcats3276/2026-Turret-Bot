package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.LeftShooterSS.LeftShooterSetpoints;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterSS.RightShooterSetpoints;

public final class Constants {

    public static final Distance ROBOT_WIDTH = Meters.of(0.932);
    public static final Distance ROBOT_LENGTH = Meters.of(0.776288);

    public static class FieldConstants {
        public static final Distance FIELD_LENGTH = Inches.of(651.2);
        public static final Distance FIELD_WIDTH = Inches.of(317.7);

        public static boolean isValidFieldTranslation(Translation3d translation) {
          return isValidFieldTranslation(translation.toTranslation2d());
        }

        public static boolean isValidFieldTranslation(Translation2d translation) {
          return translation.getX() >= 0.0 && translation.getX() <= FIELD_LENGTH.in(Meters) && translation.getY() >= 0.0
              && translation.getY() <= FIELD_WIDTH.in(Meters);
        }
    } 

    public static final double stickDeadband = 0.1;

    public static final class RobotConstants{

        public static class VisionConstants {
            public static final Distance FIELD_LENGTH = Inches.of(651.2);
            public static final Distance FIELD_WIDTH = Inches.of(317.7);
            public static final String[] APRILTAG_CAMERA_NAMES = { "limelight-l", "limelight-r" };

            public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
                new Transform3d(
                    new Translation3d(Inches.of(10.93669038), Inches.of(12.47240505), Inches.of(6.7451498)),
                    new Rotation3d(0.0, degreesToRadians(20), -90)),

                new Transform3d(
                    new Translation3d(Inches.of(10.93669038), Inches.of(-12.47240505), Inches.of(6.7451498)),
                    new Rotation3d(0.0, degreesToRadians(20), 90)),

                // new Transform3d(
                //     new Translation3d(Inches.of(-10.050), Inches.of(-11.04), Inches.of(12.015)),
                //     new Rotation3d(Math.PI, degreesToRadians(28), Math.PI)),
            };

            public static final int LIMELIGHT_BLUE_PIPELINE = 0;

            // The standard deviations of our vision estimated poses, which affect correction rate
            public static final double APRILTAG_STD_DEVS = 0.05;

            /** The max average distance for AprilTag measurements to be considered valid */
            public static final Distance TAG_DISTANCE_THRESHOLD = Meters.of(3.5);

            /** The max distance from the starting pose for AprilTag measurements to be considered valid */
            public static final Distance STARTING_DISTANCE_THRESHOLD = Meters.of(3.0);

            /** The robot angular velocity threshold for accepting vision measurements */
            public static final AngularVelocity ANGULAR_VELOCITY_THRESHOLD = DegreesPerSecond.of(720);

            public static final Translation2d TARGET_BLUE = new Translation2d(Inches.of(182.143595), Inches.of(158.84375));
                
            /** Translation of the hub on the red side */
            public static final Translation2d TARGET_RED = new Translation2d(Inches.of(469.078905), Inches.of(158.84375));

            public static final Translation2d SHUTTLE_BLUE_LEFT = new Translation2d(
                Inches.of(2.0),
                FIELD_WIDTH.minus(Inches.of(64)));
            /** Translations for shuttling on the blue side, with Z < 1/2 of the field */
            public static final Translation2d SHUTTLE_BLUE_RIGHT = new Translation2d(Inches.of(2.0), Inches.of(64));

            /** Translations for shuttling on the red side, with Z > 1/2 of the field */
            public static final Translation2d SHUTTLE_RED_RIGHT = FlippingUtil.flipFieldPosition(SHUTTLE_BLUE_RIGHT);
            /** Translations for shuttling on the red side, with Z < 1/2 of the field */
            public static final Translation2d SHUTTLE_RED_LEFT = FlippingUtil.flipFieldPosition(SHUTTLE_BLUE_LEFT);

            public static final Distance DANGER_ZONE_MIN_BLUE = Meters.of(4.15);
            public static final Distance DANGER_ZONE_MAX_BLUE = Meters.of(5.25);
            public static final Distance DANGER_ZONE_MIN_RED = FIELD_LENGTH.minus(DANGER_ZONE_MAX_BLUE);
            public static final Distance DANGER_ZONE_MAX_RED = FIELD_LENGTH.minus(DANGER_ZONE_MIN_BLUE);
            public static final Distance AUTO_SHOOTER_BARRIER_BLUE = (DANGER_ZONE_MIN_BLUE.plus(DANGER_ZONE_MAX_BLUE)).div(2);
            public static final Distance AUTO_SHOOTER_BARRIER_RED = FIELD_LENGTH.minus(AUTO_SHOOTER_BARRIER_BLUE);
        }   
        
        public static final class FrontLeftTurret{
            /*Motors for Turret */
            public static final int Shoot_Motor = 51;
            public static final int Hood_Rotation_Motor = 52;
            public static final int Turret_Rotation_Motor = 50;
            /*Encoders for Turret */
            public static final int Turret_Rotation_Encoder = 5;
            public static final int Hood_Encoder = 7;
            /*MAX SPEED DONT CHANGE */
            public static final double MAX_SPEED = .35;
            public static final double MAX_SPEED_Auto = .50;

            public static final Angle YAW_LIMIT_FORWARD = Rotations.of(.5);
            public static final Angle YAW_LIMIT_REVERSE = Rotations.of(-.5);

            public static final Angle YAW_RANGE_FORWARD = YAW_LIMIT_FORWARD
                .minus(YAW_LIMIT_FORWARD.minus(YAW_LIMIT_REVERSE).minus(Rotations.one()).div(2.0));
            public static final Angle YAW_RANGE_REVERSE = YAW_LIMIT_REVERSE
                .plus(YAW_LIMIT_FORWARD.minus(YAW_LIMIT_REVERSE).minus(Rotations.one()).div(2.0));

            public static final Angle PITCH_LIMIT_FORWARD = Degrees.of(54.7);
            public static final Angle PITCH_LIMIT_REVERSE = Degrees.of(25);
            public static final Angle PITCH_HOME_ANGLE = PITCH_LIMIT_REVERSE;

            public static final Translation2d ROBOT_TO_LEFT_SHOOTER = new Translation2d(Inches.of(6.5), Inches.of(7.126));

            // public static final double FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER = 0.29;
            public static final Angle FUEL_EXIT_ANGLE_OFFSET = Degrees.of(65.0);

            private static InterpolatingTreeMap<Double, LeftShooterSS.LeftShooterSetpoints> createLeftShooterShuttleInterpolator(){
                var map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LeftShooterSetpoints::interpolate);

                map.put(
                    2.5, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(45), 
                            //ShotSpeed
                            RotationsPerSecond.of(15)));
                map.put(
                    16.5, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(54.5), 
                            //ShotSpeed
                            RotationsPerSecond.of(80)));
                return map;
            }

            public static final InterpolatingTreeMap<Double, LeftShooterSS.LeftShooterSetpoints> Shuttle_SetPoints_Left = createLeftShooterShuttleInterpolator();

            private static InterpolatingTreeMap<Double, LeftShooterSS.LeftShooterSetpoints> createLeftShooterInterpolator(){
                var map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LeftShooterSetpoints::interpolate);

                map.put(
                    1.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(25), 
                            //ShotSpeed
                            RotationsPerSecond.of(31)));
                map.put(
                    1.5, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(25), 
                            //ShotSpeed
                            RotationsPerSecond.of(34.5)));
                map.put(
                    2.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(25), 
                            //ShotSpeed
                            RotationsPerSecond.of(36)));
                map.put(
                    2.5, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(27), 
                            //ShotSpeed
                            RotationsPerSecond.of(38)));
                map.put(
                    3.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(29), 
                            //ShotSpeed
                            RotationsPerSecond.of(40)));
                map.put(
                    3.5, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(32), 
                            //ShotSpeed
                            RotationsPerSecond.of(42)));
                map.put(
                    4.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(36), 
                            //ShotSpeed
                            RotationsPerSecond.of(44)));
                map.put(
                    4.5, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(38), 
                            //ShotSpeed
                            RotationsPerSecond.of(45)));
                map.put(
                    5.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(40), 
                            //ShotSpeed
                            RotationsPerSecond.of(46)));
                map.put(
                    5.7, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Degrees.of(43), 
                            //ShotSpeed
                            RotationsPerSecond.of(47)));
                return map;
            }

            public static final InterpolatingTreeMap<Double, LeftShooterSS.LeftShooterSetpoints> Hub_SetPoints_Left = createLeftShooterInterpolator();
        }

        public static final class FrontRightTurret{
            /*Motors for Turret */
            public static final int Shoot_Motor = 61;
            public static final int Hood_Rotation_Motor = 62;
            public static final int Turret_Rotation_Motor = 60;
            /*Encoders for Turret */
            public static final int Turret_Rotation_Encoder = 6;

            public static final int Hood_Encoder = 8;
            /*MAX SPEED DONT CHANGE */
            public static final double MAX_SPEED = .35;
            public static final double MAX_SPEED_Auto = .50;

            public static final Angle YAW_LIMIT_FORWARD = Rotations.of(.5);
            public static final Angle YAW_LIMIT_REVERSE = Rotations.of(-.5);

            public static final Angle YAW_RANGE_FORWARD = YAW_LIMIT_FORWARD
                .minus(YAW_LIMIT_FORWARD.minus(YAW_LIMIT_REVERSE).minus(Rotations.one()).div(2.0));
            public static final Angle YAW_RANGE_REVERSE = YAW_LIMIT_REVERSE
                .plus(YAW_LIMIT_FORWARD.minus(YAW_LIMIT_REVERSE).minus(Rotations.one()).div(2.0));

            public static final Angle PITCH_LIMIT_FORWARD = Degrees.of(54.7);
            public static final Angle PITCH_LIMIT_REVERSE = Degrees.of(25);
            public static final Angle PITCH_HOME_ANGLE = PITCH_LIMIT_REVERSE;

            public static final Translation2d ROBOT_TO_Right_SHOOTER = new Translation2d(Inches.of(6.5), Inches.of(-7.126));

            public static final double FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER = 0.08;
            public static final Angle FUEL_EXIT_ANGLE_OFFSET = Degrees.of(75.0);

            private static InterpolatingTreeMap<Double, RightShooterSS.RightShooterSetpoints> createRightShooterShuttleInterpolator(){
                var map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), RightShooterSetpoints::interpolate);

                map.put(
                    2.5, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(45), 
                            //ShotSpeed
                            RotationsPerSecond.of(15)));
                map.put(
                    16.5, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(54.5), 
                            //ShotSpeed
                            RotationsPerSecond.of(80)));
                return map;
            }

            public static final InterpolatingTreeMap<Double, RightShooterSS.RightShooterSetpoints> Shuttle_SetPoints_Right = createRightShooterShuttleInterpolator();

            private static InterpolatingTreeMap<Double, RightShooterSS.RightShooterSetpoints> createRightShooterInterpolator(){
                var map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), RightShooterSetpoints::interpolate);

                map.put(
                    1.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(25), 
                            //ShotSpeed
                            RotationsPerSecond.of(31)));
                map.put(
                    1.5, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(25), 
                            //ShotSpeed
                            RotationsPerSecond.of(34.5)));
                map.put(
                    2.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(25), 
                            //ShotSpeed
                            RotationsPerSecond.of(36)));
                map.put(
                    2.5, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(27), 
                            //ShotSpeed
                            RotationsPerSecond.of(38)));
                map.put(
                    3.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(29), 
                            //ShotSpeed
                            RotationsPerSecond.of(40)));
                map.put(
                    3.5, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(32), 
                            //ShotSpeed
                            RotationsPerSecond.of(42)));
                map.put(
                    4.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(36), 
                            //ShotSpeed
                            RotationsPerSecond.of(44)));
                map.put(
                    4.5, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(38), 
                            //ShotSpeed
                            RotationsPerSecond.of(45)));
                map.put(
                    5.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(40), 
                            //ShotSpeed
                            RotationsPerSecond.of(46)));
                map.put(
                    5.7, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Degrees.of(43), 
                            //ShotSpeed
                            RotationsPerSecond.of(47)));
                return map;
            }

            public static final InterpolatingTreeMap<Double, RightShooterSS.RightShooterSetpoints> Hub_SetPoints_Right = createRightShooterInterpolator();
        }

        public static final class Feeder{
            /*Motors for Feeder */
            public static final int Feeder_Motor = 46;

            public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1);
        }

        public static final class Indexer{
            /*Motors for Belt System */
            public static final int Indexer_Motor_Left = 44;
            public static final int Indexer_Motor_Right = 45;

            public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1);
        }

        public static final class Infeed{
            /*Motors for Infeed */
            public static final int Infeed_Motor = 41;
            public static final int Infeed_Rotation_Motor_Left = 42;
            // public static final int Infeed_Rotation_Motor_Right = 43;
            /*Encoders for Infeed*/
            public static final int Infeed_Rotation_Encoder = 4;

            public static final double Max_Speed = 1;
            public static final double Infeed_POS = -.30;
            public static final double FullComp = .3;



            public static final double Infeed_POS2 = -.10;


            public static final double Shooting_POS = -.35;
            // public static final double Infeed_Compliance_Pos = .11;

        }
    }

    public static final class Swerve {

        public static final String[] cameraNames = {"limelight-1", "limelight-2", "limelight-3", "limelight-4"};

        public static final int pigeonID = 5;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK5n.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK5n.driveRatios.R3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.75);
        public static final double wheelBase = Units.inchesToMeters(21.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


        public static final Translation2d mod0Offset = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d mod1Offset = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d mod2Offset = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d mod3Offset = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentThreshold = 60;
        // public static final double driveCurrentThresholdTime = 0.1;
        public static final double driveCurrentThresholdTime = 1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.15; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(179.3848); //26.279
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(15.996); //-130.253
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(97.295); //14.238
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-57.041);//-60.029, -146.777, -156.972
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 6.065;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.4;
        public static final double kMaxAngularSpeedRadiansPerSecond = -Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = -Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
