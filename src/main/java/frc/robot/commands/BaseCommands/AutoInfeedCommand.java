package frc.robot.commands.BaseCommands;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static edu.wpi.first.wpilibj.util.Color.kRed;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedPivotSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftShooterSS.LeftShooterSetpoints;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;
import frc.robot.subsystems.SwerveSS;
import frc.robot.subsystems.RightShooterSS.RightShooterSetpoints;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER;
import static frc.robot.Constants.RobotConstants.VisionConstants.DANGER_ZONE_MAX_BLUE;
import static frc.robot.Constants.RobotConstants.VisionConstants.DANGER_ZONE_MAX_RED;
import static frc.robot.Constants.RobotConstants.VisionConstants.DANGER_ZONE_MIN_BLUE;
import static frc.robot.Constants.RobotConstants.VisionConstants.DANGER_ZONE_MIN_RED;
import static frc.robot.Constants.RobotConstants.VisionConstants.AUTO_SHOOTER_BARRIER_BLUE;
import static frc.robot.Constants.RobotConstants.VisionConstants.AUTO_SHOOTER_BARRIER_RED;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.Hub_SetPoints_Left;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.Hub_SetPoints_Right;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.Shuttle_SetPoints_Left;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.Shuttle_SetPoints_Right;

import java.util.function.Function;
import java.util.function.Supplier;

/**
 * This command automatically shoots at a target until interrupted. It takes current robot velocity into account to
 * "shoot on the move". This command must be interrupted to stop, most likely by the driver releasing a button or by an
 * autonomous routine setting an end condition.
 * <p>
 * This command has configurability to be used for both shooting at the hub and shooting while shuttling fuel across the
 * field. The target to shoot at is determined by a function, and the shooter settings lookup table is a parameter.
 */
public class AutoInfeedCommand extends Command {

  private final RightShooterSS s_RightShooter;
  private final RightTurretSS s_RightTurret;
  private final RightShooterHoodSS s_RightShooterHood;
  private final LeftShooterSS s_LeftShooter;
  private final LeftTurretSS s_LeftTurret;
  private final LeftShooterHoodSS s_LeftShooterHood;
  private final FeederSS s_Feeder;
  private final IndexerSS s_Indexer;
  private final InfeedSS s_Infeed;
  private final SwerveSS s_Swerve;
  private final InfeedPivotSS s_InfeedPivot;
  private final Function<Translation2d, Translation2d> targetSelector;

  // Reusable object to prevent reallocation (to reduce memory pressure)
  private final MutAngle RightTurretYawTarget = Rotations.mutable(0);
  private final MutAngle LeftTurretYawTarget = Rotations.mutable(0);

  private boolean isShooting;

  /**
   * Constructor
   * 
   * @param RightShooterSS shooter subsystem
   * @param FeederSS feeder subsystem
   * @param indexerSS spindexer subsystem
   * @param robotLedSubsystem led subsystem
   * @param targetSelector function that takes the shooter's translation and returns the target translation to shoot at
   * @param lookupTableR lookup table mapping distance to shooter setpoints
   */
  public AutoInfeedCommand(
      RightShooterSS s_RightShooter,
      RightTurretSS s_RightTurret,
      RightShooterHoodSS s_RightShooterHood,
      LeftShooterSS s_LeftShooter,
      LeftTurretSS s_LeftTurret,
      LeftShooterHoodSS s_LeftShooterHood,
      FeederSS s_Feeder,
      IndexerSS s_Indexer,
      InfeedSS s_Infeed,
      SwerveSS s_Swerve,
      InfeedPivotSS s_InfeedPivot,
      Function<Translation2d, Translation2d> targetSelector
      ) {
    this.s_RightShooter = s_RightShooter;
    this.s_RightTurret = s_RightTurret;
    this.s_RightShooterHood = s_RightShooterHood;
    this.s_LeftShooter = s_LeftShooter;
    this.s_LeftTurret = s_LeftTurret;
    this.s_LeftShooterHood = s_LeftShooterHood;
    this.s_Feeder = s_Feeder;
    this.s_Indexer = s_Indexer;
    this.s_Infeed = s_Infeed;
    this.s_Swerve = s_Swerve;
    this.s_InfeedPivot = s_InfeedPivot;
    this.targetSelector = targetSelector;

    addRequirements(s_RightShooter, s_Feeder, s_Indexer, s_RightTurret, s_RightShooterHood, s_Infeed, s_LeftShooter, s_LeftShooterHood, s_LeftTurret);
  }

  @Override
  public void initialize() {
    // Reset states
    isShooting = false;
  }

  @Override
  public void execute() {

    s_Infeed.SetSpeed(RotationsPerSecond.of(1));
    s_InfeedPivot.PID(Constants.RobotConstants.Infeed.Infeed_POS, Constants.RobotConstants.Infeed.Max_Speed);
    var robotPose = s_Swerve.swerveOdometry.getEstimatedPosition();
    // var currentChassisSpeeds = s_Swerve.getCurrentFieldChassisSpeeds();
    var currentChassisSpeeds = s_Swerve.getRobotSpeed();
    
    // Translation of the shooter on the field (used for distance/angle calculations).
    var rightShooterTranslation = RightShooterSS.getShooterTranslation(robotPose);
    var leftShooterTranslation = LeftShooterSS.getShooterTranslation(robotPose);

    // Get the target to shoot at
    var rightTargetTranslation = targetSelector.apply(rightShooterTranslation);
    var leftTargetTranslation = targetSelector.apply(leftShooterTranslation);
    var rightActualTargetDistance = rightShooterTranslation.getDistance(rightTargetTranslation);
    var leftActualTargetDistance = leftShooterTranslation.getDistance(leftTargetTranslation);


    
    // If the shooter is under the trench or over the bump, don't shoot
    var rightShooterX = rightShooterTranslation.getX();
    var leftShooterX = rightShooterTranslation.getX();
    InterpolatingTreeMap<Double, LeftShooterSetpoints> lookupTableL;
    InterpolatingTreeMap<Double, RightShooterSetpoints> lookupTableR;
    
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue){
      // if ((rightShooterX < AUTO_SHOOTER_BARRIER_BLUE.in(Meters))
      //  || (leftShooterX < AUTO_SHOOTER_BARRIER_BLUE.in(Meters))){
        lookupTableR = Hub_SetPoints_Right;
        lookupTableL = Hub_SetPoints_Left;
      // }
      // else {
      //   lookupTableR = Shuttle_SetPoints_Right;
      //   lookupTableL = Shuttle_SetPoints_Left;
      // }
    }
    else{
      // if ((rightShooterX > AUTO_SHOOTER_BARRIER_RED.in(Meters))
      //  || (leftShooterX > AUTO_SHOOTER_BARRIER_RED.in(Meters))){
        lookupTableR = Hub_SetPoints_Right;
        lookupTableL = Hub_SetPoints_Left;
      // }
      // else {
      //   lookupTableR = Shuttle_SetPoints_Right;
      //   lookupTableL = Shuttle_SetPoints_Left;
      // }
    }

    // if ((rightShooterX > DANGER_ZONE_MIN_BLUE.in(Meters) && rightShooterX < DANGER_ZONE_MAX_BLUE.in(Meters))
    //  || (rightShooterX > DANGER_ZONE_MIN_RED.in(Meters) && rightShooterX < DANGER_ZONE_MAX_RED.in(Meters))
    //  || (leftShooterX > DANGER_ZONE_MIN_BLUE.in(Meters) && leftShooterX < DANGER_ZONE_MAX_BLUE.in(Meters))
    //  || (leftShooterX > DANGER_ZONE_MIN_RED.in(Meters) && leftShooterX < DANGER_ZONE_MAX_RED.in(Meters))){
    //   s_RightShooterHood.stowPitch();
    //   s_LeftShooterHood.stowPitch();
    //   return;
    // }

    // 1. Compute the velocity of the fuel at the shooter's location on the field.
    //
    // - When the robot (or turret) is moving, the fuel inherits that motion at the moment it leaves the flywheel. To
    // predict where the fuel will land we must account for that initial velocity.
    // - There are two contributions:
    // A) vRobot: the robot's linear translation velocity
    // B) vRightTan: tangential velocity at the shooter from angular rotation acting on the shooter
    //
    // The sum of these gives the initial velocity of the fuel relative to the field, which we use to compute how far
    // the projectile will shift during its flight.
    var vRobot = new Translation2d(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);

    // Total angular rate that affects the shooter (robot yaw + turret yaw rate).
    var omega = currentChassisSpeeds.omegaRadiansPerSecond;

    // Convert rotation about the robot center into a tangential linear velocity at the shooter.
    // This creates a tangential velocity at the shooter mount because the shooter is offset from the robot center.
    var robotToRightShooter = rightShooterTranslation.minus(robotPose.getTranslation());
    var robotToLeftShooter = leftShooterTranslation.minus(robotPose.getTranslation());
    var vRightTan = new Translation2d(-omega * robotToRightShooter.getY(), omega * robotToRightShooter.getX());
    var vLeftTan = new Translation2d(-omega * robotToLeftShooter.getY(), omega * robotToLeftShooter.getX());

    // Effective shooter velocity = robot linear + tangential from rotation.
    var effectiveRightShooterVelocity = vRobot.plus(vRightTan);
    var effectiveLeftShooterVelocity = vRobot.plus(vLeftTan);

    // 2. Iteratively solve for the aim point that compensates for time-of-flight.
    //
    // Rationale:
    // - The flight time depends on the flywheel speed and rpitch, which are chosen based on the distance to the target.
    // But the distance depends on the offset caused by the robot's motion during flight — a circular dependency.
    // - We therefore perform a few iterations: estimate distance -> choose shooter setpoints -> compute flight time ->
    // predict where the target will appear after that flight -> repeat.
    var predictedRightTargetTranslation = rightTargetTranslation;
    var predictedLeftTargetTranslation = leftTargetTranslation;
    Translation2d targetRightPredictedOffset = new Translation2d();
    Translation2d targetLeftPredictedOffset = new Translation2d();

    RightShooterSetpoints RightshootingSettings;
    LeftShooterSetpoints LeftshootingSettings;
    for (int i = 0; i < 4; i++) {
      // Distance from the shooter to the currently predicted target point.
      // This distance drives the lookup table which selects rpitch and flywheel speed.
      var predictedRightTargetDistance = rightShooterTranslation.getDistance(predictedRightTargetTranslation);
      var predictedLeftTargetDistance = leftShooterTranslation.getDistance(predictedLeftTargetTranslation);
      RightshootingSettings = lookupTableR.get(predictedRightTargetDistance);
      LeftshootingSettings = lookupTableL.get(predictedLeftTargetDistance);


      var RTimeUntilScored = 0.0;
      var LTimeUntilScored = 0.0;
      var rrps = RightshootingSettings.shotVelocity().in(RotationsPerSecond);
      var lrps = LeftshootingSettings.shotVelocity().in(RotationsPerSecond);
      var rpitch = RightshootingSettings.shotAngle();
      var lpitch = LeftshootingSettings.shotAngle();

      // Compute how far the fuel initial velocity (from robot motion) will shift the target point.
      // We subtract this offset from the real target position to produce a "lead" point to aim at.
      //
      // - If the robot is moving forward, the fuel will land further forward relative to a stationary shot. To hit the
      // desired (static) target we need to aim backwards relative to the instantaneous target position by the amount
      // the fuel will be carried during flight.
      targetRightPredictedOffset = effectiveRightShooterVelocity.times(-RTimeUntilScored);
      targetLeftPredictedOffset = effectiveLeftShooterVelocity.times(-LTimeUntilScored);
      predictedRightTargetTranslation = rightTargetTranslation.plus(targetRightPredictedOffset);
      predictedLeftTargetTranslation = leftTargetTranslation.plus(targetLeftPredictedOffset);

      SmartDashboard.putNumber("Target distance Right", predictedRightTargetDistance);
      SmartDashboard.putNumber("Target distance Left", predictedLeftTargetDistance);
      SmartDashboard.putNumber("Right Shooter Pitch", s_RightShooterHood.getFuelPitch(rpitch).in(Degrees));
      SmartDashboard.putNumber("Left Shooter Pitch", s_LeftShooterHood.getFuelPitch(lpitch).in(Degrees));
    }

    // After iterating, resolve final shooter setpoints for the converged predicted target.
    RightshootingSettings = lookupTableR.get(predictedRightTargetTranslation.getDistance(rightShooterTranslation));
    LeftshootingSettings = lookupTableL.get(predictedLeftTargetTranslation.getDistance(leftShooterTranslation));

    // Command the shooter pitch, yaw, and flywheel speed from the lookup table.
    s_RightTurret.stowYaw();
    // s_RightShooterHood.LinearActuator(RightshootingSettings.shotAngle());
    s_RightShooterHood.stowPitch();
    s_RightShooter.setSpeed(RightshootingSettings.shotVelocity());
    s_LeftTurret.stowYaw();
    // s_LeftShooterHood.LinearActuator(LeftshootingSettings.shotAngle());
    s_LeftShooterHood.stowPitch();
    s_LeftShooter.setSpeed(LeftshootingSettings.shotVelocity());

    // s_Infeed.SetSpeed(RotationsPerSecond.of(1));
    // s_InfeedPivot.PID(Constants.RobotConstants.Infeed.Infeed_POS, Constants.RobotConstants.Infeed.Max_Speed);

    s_Feeder.Stop();
    s_Indexer.Stop();

  }

  @Override
  public void end(boolean interrupted) {
  }
}