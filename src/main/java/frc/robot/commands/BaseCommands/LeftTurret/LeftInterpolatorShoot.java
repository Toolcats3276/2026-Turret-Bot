package frc.robot.commands.BaseCommands.LeftTurret;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.Hub_SetPoints_By_Limelight_Degrees_Left;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftShooterSS.LeftShooterSetpoints;


/*
 * Command to shoot fuel without aiming. It will shoot at a fixed yaw, pitch, and velocity
 */
public class LeftInterpolatorShoot extends Command {
  private final IndexerSS s_Indexer;
  private final LeftShooterHoodSS s_LeftShooterHood;
  private final LeftShooterSS s_LeftShooter;
  private final FeederSS s_Feeder;
  private final InfeedSS s_Infeed;
  private final LeftShooterSetpoints setpoints;
  private boolean isShooting = false;

  /**
   * Constructor for ShootCommand
   * 
   * @param s_Indexer the spindexer subsystem
   * @param LeftShooterHoodSS the feeder subsystem
   * @param s_LeftShooter the shooter subsystem
   * @param targetDistance the distance of the target to use for setpoints
   */
  public LeftInterpolatorShoot(IndexerSS s_Indexer, LeftShooterHoodSS s_LeftShooterHood, LeftShooterSS s_LeftShooter, FeederSS s_Feeder, InfeedSS s_Infeed, Angle targetDistance) {
    this.s_LeftShooterHood = s_LeftShooterHood;
    this.s_Indexer = s_Indexer;
    this.s_LeftShooter = s_LeftShooter;
    this.s_Feeder = s_Feeder;
    this.s_Infeed = s_Infeed;

    setpoints = Hub_SetPoints_By_Limelight_Degrees_Left.get(targetDistance.in(Degrees));

    addRequirements(s_LeftShooterHood, s_Indexer, s_LeftShooter, s_Feeder);
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
    s_LeftShooterHood.LinearActuator(setpoints.shotAngle());;
    s_LeftShooter.setSpeed(setpoints.shotVelocity());

  }

  public void end(boolean interrupted) {
    s_Indexer.Stop();
    s_LeftShooterHood.Stop();
    s_LeftShooter.Stop();
  }
}