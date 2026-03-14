package frc.robot;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.LeftShooterSS.LeftShooterSetpoints;
import frc.robot.subsystems.RightShooterSS.RightShooterSetpoints;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class RobotConstants{

        public static final class FrontLeftTurret{
            /*Motors for Turret */
            public static final int Shoot_Motor_Left_Motor = 51;
            public static final int Shoot_Motor_Right_Motor = 52;
            public static final int Turret_Rotation_Motor = 50;
            /*Encoders for Turret */
            public static final int Turret_Rotation_Encoder = 5;
            /*MAX SPEED DONT CHANGE */
            public static final double MAX_SPEED = .35;
            public static final double MAX_SPEED_Auto = .50;

            private static InterpolatingTreeMap<Double, LeftShooterSS.LeftShooterSetpoints> createLeftShooterInterpolator(){
                var map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LeftShooterSetpoints::interpolate);

                map.put(
                    0.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0), 
                            //ShotSpeed
                            RotationsPerSecond.of(.6)));
                map.put(
                    2.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.61)));
                map.put(
                    4.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.61)));
                map.put(
                    6.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.615)));
                map.put(
                    8.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.6475)));
                map.put(
                    10.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.678)));
                map.put(
                    12.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.682)));
                map.put(
                    14.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.69)));
                map.put(
                    16.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.70)));
                map.put(
                    18.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.1), 
                            //ShotSpeed
                            RotationsPerSecond.of(.71)));
                map.put(
                    20.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.15), 
                            //ShotSpeed
                            RotationsPerSecond.of(.71)));
                map.put(
                    22.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.15), 
                            //ShotSpeed
                            RotationsPerSecond.of(.715)));
                map.put(
                    24.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.2), 
                            //ShotSpeed
                            RotationsPerSecond.of(.73)));
                map.put(
                    26.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.25), 
                            //ShotSpeed
                            RotationsPerSecond.of(.74)));
                map.put(
                    28.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.30), 
                            //ShotSpeed
                            RotationsPerSecond.of(.75)));
                map.put(
                    30.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.3), 
                            //ShotSpeed
                            RotationsPerSecond.of(.775)));
                map.put(
                    32.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.4), 
                            //ShotSpeed
                            RotationsPerSecond.of(.8125)));
                map.put(
                    35.0, 
                        new LeftShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.72), 
                            //ShotSpeed
                            RotationsPerSecond.of(1)));
                return map;
            }

            public static final InterpolatingTreeMap<Double, LeftShooterSS.LeftShooterSetpoints> Hub_SetPoints_By_Limelight_Degrees_Left = createLeftShooterInterpolator();
        }

        public static final class FrontRightTurret{
            /*Motors for Turret */
            public static final int Shoot_Motor_Left_Motor = 61;
            public static final int Shoot_Motor_Right_Motor = 62;
            public static final int Turret_Rotation_Motor = 60;
            /*Encoders for Turret */
            public static final int Turret_Rotation_Encoder = 6;
            /*MAX SPEED DONT CHANGE */
            public static final double MAX_SPEED = .35;
            public static final double MAX_SPEED_Auto = .50;



            private static InterpolatingTreeMap<Double, RightShooterSS.RightShooterSetpoints> createRightShooterInterpolator(){
                var map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), RightShooterSetpoints::interpolate);

                map.put(
                    0.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0), 
                            //ShotSpeed
                            RotationsPerSecond.of(.60)));
                map.put(
                    2.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.61)));
                map.put(
                    4.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.61)));
                map.put(
                    6.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.615)));
                map.put(
                    8.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.6475)));
                map.put(
                    10.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.678)));
                map.put(
                    12.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.682)));
                map.put(
                    14.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.69)));
                map.put(
                    16.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.05), 
                            //ShotSpeed
                            RotationsPerSecond.of(.70)));
                map.put(
                    18.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.1), 
                            //ShotSpeed
                            RotationsPerSecond.of(.71)));
                map.put(
                    20.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.15), 
                            //ShotSpeed
                            RotationsPerSecond.of(.71)));
                map.put(
                    22.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.15), 
                            //ShotSpeed
                            RotationsPerSecond.of(.72)));
                map.put(
                    24.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.2), 
                            //ShotSpeed
                            RotationsPerSecond.of(.74)));
                map.put(
                    26.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.25), 
                            //ShotSpeed
                            RotationsPerSecond.of(.74)));
                map.put(
                    28.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.30), 
                            //ShotSpeed
                            RotationsPerSecond.of(.75)));
                map.put(
                    30.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.3), 
                            //ShotSpeed
                            RotationsPerSecond.of(.775)));
                map.put(
                    32.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.4), 
                            //ShotSpeed
                            RotationsPerSecond.of(.8125)));
                map.put(
                    35.0, 
                        new RightShooterSetpoints(
                            //Shot Angle
                            Millimeter.of(0.72), 
                            //ShotSpeed
                            RotationsPerSecond.of(1)));
                return map;
            }

            public static final InterpolatingTreeMap<Double, RightShooterSS.RightShooterSetpoints> Hub_SetPoints_By_Limelight_Degrees_Right = createRightShooterInterpolator();
        }

        public static final class Feeder{
            /*Motors for Feeder */
            public static final int Feeder_Motor = 45;

            public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1);
        }

        public static final class Indexer{
            /*Motors for Belt System */
            public static final int Indexer_Motor_Left = 44;

            public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1);
        }

        public static final class Infeed{
            /*Motors for Infeed */
            public static final int Infeed_Motor = 41;
            public static final int Infeed_Rotation_Motor_Left = 42;
            public static final int Infeed_Rotation_Motor_Right = 43;
            /*Encoders for Infeed*/
            public static final int Infeed_Rotation_Encoder = 4;

            public static final double Max_Speed = 1;
            public static final double Infeed_POS = -.29;


            public static final double Infeed_POS2 = -.13;


            public static final double Shooting_POS = -.24;
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
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
