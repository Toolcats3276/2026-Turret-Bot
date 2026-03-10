package frc.robot.commands.BaseCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.RobotConstants.FrontRightTurret.Hub_SetPoints_By_Limelight_Degrees_Right;
import static frc.robot.Constants.RobotConstants.FrontLeftTurret.Hub_SetPoints_By_Limelight_Degrees_Left;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftShooterSS.LeftShooterSetpoints;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightShooterSS.RightShooterSetpoints;

public class InterpolatorShootCommand extends Command {

  private final IndexerSS s_Indexer;
  private final RightShooterHoodSS s_RightShooterHood;
  private final LeftShooterHoodSS s_LeftShooterHood;
  private final RightShooterSS s_RightShooter;
  private final LeftShooterSS s_LeftShooter;
  private final FeederSS s_Feeder;
  private final InfeedSS s_Infeed;
  private RightShooterSetpoints setpointsRight;
    private LeftShooterSetpoints setpointsLeft;
    private boolean isShooting = false;
  
    public InterpolatorShootCommand(IndexerSS s_Indexer, InfeedSS s_Infeed, RightShooterHoodSS s_RightShooterHood, LeftShooterHoodSS s_LeftShooterHood, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, FeederSS s_Feeder) {
      this.s_RightShooterHood = s_RightShooterHood;
      this.s_LeftShooterHood = s_LeftShooterHood;
      this.s_Indexer = s_Indexer;
      this.s_RightShooter = s_RightShooter;
      this.s_LeftShooter = s_LeftShooter;
      this.s_Feeder = s_Feeder;
      this.s_Infeed = s_Infeed;
  
      setpointsRight = Hub_SetPoints_By_Limelight_Degrees_Right.get(s_RightShooter.TyValue());
      setpointsLeft = Hub_SetPoints_By_Limelight_Degrees_Left.get(s_LeftShooter.TyValue());
  
  
      addRequirements( s_Indexer, s_Infeed, s_RightShooterHood, s_LeftShooterHood, s_RightShooter, s_LeftShooter, s_Feeder);
    }
  
    @Override
    public void initialize() {
      isShooting = false;
    }
  
    @Override
    public void execute() {

      SmartDashboard.putNumber("Right Shooter Setpoints", setpointsRight.shotVelocity().in(RotationsPerSecond));
      SmartDashboard.putNumber("Left Shooter Setpoints", setpointsLeft.shotVelocity().in(RotationsPerSecond));
  
      setpointsRight = Hub_SetPoints_By_Limelight_Degrees_Right.get(s_RightShooter.TyValue());
      setpointsLeft = Hub_SetPoints_By_Limelight_Degrees_Left.get(s_LeftShooter.TyValue());
        /*
     * sets the Yaw, Pitch, and Angle
     */
    s_RightShooterHood.LinearActuator(setpointsRight.shotAngle());
    s_RightShooter.setSpeed(setpointsRight.shotVelocity());
    s_LeftShooterHood.LinearActuator(setpointsLeft.shotAngle());
    s_LeftShooter.setSpeed(setpointsLeft.shotVelocity());
    /*
     * Checks to make sure the shooter is ready and up to speed
     * before runnig the spindexer and feeder
     */
    if (isShooting || s_RightShooter.isReadyToShoot()) {
      isShooting = true;
      s_Indexer.setSpeed(setpointsRight.indexerVelocity());
      s_Feeder.setSpeed(setpointsRight.feederVelocity());
      s_Infeed.SetSpeed(setpointsRight.infeedVelocity());
    }
  }

  @Override
  public void end(boolean interrupted) {
    s_Indexer.Stop();
    s_Feeder.Stop();
    s_Infeed.Stop();
    s_RightShooterHood.Stop();
    s_LeftShooterHood.Stop();
    s_RightShooter.Stop();
    s_LeftShooter.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
