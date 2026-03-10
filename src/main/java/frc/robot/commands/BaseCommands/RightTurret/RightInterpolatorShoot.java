package frc.robot.commands.BaseCommands.RightTurret;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.Hub_SetPoints_By_Limelight_Degrees_Left;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.Hub_SetPoints_By_Limelight_Degrees_Right;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightShooterSS.RightShooterSetpoints;


/*
 * Command to shoot fuel without aiming. It will shoot at a fixed yaw, pitch, and velocity
 */
public class RightInterpolatorShoot extends Command {
  private final IndexerSS s_Indexer;
  private final RightShooterHoodSS s_RightShooterHood;
  private final RightShooterSS s_RightShooter;
  private final FeederSS s_Feeder;
  private final InfeedSS s_Infeed;
  private final RightShooterSetpoints setpoints;
  private boolean isShooting = false;

  /**
   * Constructor for ShootCommand
   * 
   * @param s_Indexer the spindexer subsystem
   * @param RightShooterHoodSS the feeder subsystem
   * @param s_RightShooter the shooter subsystem
   * @param targetDistance the distance of the target to use for setpoints
   */
  public RightInterpolatorShoot(IndexerSS s_Indexer, RightShooterHoodSS s_RightShooterHood, RightShooterSS s_RightShooter, FeederSS s_Feeder, InfeedSS s_Infeed, Angle targetDistance) {
    this.s_RightShooterHood = s_RightShooterHood;
    this.s_Indexer = s_Indexer;
    this.s_RightShooter = s_RightShooter;
    this.s_Feeder = s_Feeder;
    this.s_Infeed = s_Infeed;

    setpoints = Hub_SetPoints_By_Limelight_Degrees_Right.get(targetDistance.in(Degrees));

    addRequirements(s_RightShooterHood, s_Indexer, s_RightShooter, s_Feeder);
  }

  @Override
  public void initialize() {
    isShooting = false;
  }

  @Override
  public void execute() {
    /*
     * sets the Yaw, Pitch, and Angle
     */
    s_RightShooterHood.LinearActuator(setpoints.shotAngle());;
    s_RightShooter.setSpeed(setpoints.shotVelocity());
    /*
     * Checks to make sure the shooter is ready and up to speed
     * before runnig the spindexer and feeder
     */
    if (isShooting || s_RightShooter.isReadyToShoot()) {
      isShooting = true;
      s_Indexer.setSpeed(setpoints.indexerVelocity());
      s_Feeder.setSpeed(setpoints.feederVelocity());
      s_Infeed.SetSpeed(setpoints.infeedVelocity());
    }
  }

  public void end(boolean interrupted) {
    s_Indexer.Stop();
    s_RightShooterHood.Stop();
    s_RightShooter.Stop();
  }
}