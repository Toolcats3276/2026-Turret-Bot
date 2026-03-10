package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.BaseCommands.LeftTurret.LeftInterpolatorShoot;
import frc.robot.commands.BaseCommands.RightTurret.RightInterpolatorShoot;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterSS;

public class InterpolatorShootCoCommand extends ParallelCommandGroup {

  public InterpolatorShootCoCommand(IndexerSS s_Indexer, InfeedSS s_Infeed, RightShooterHoodSS s_RightShooterHood, LeftShooterHoodSS s_LeftShooterHood, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, FeederSS s_Feeder) {

    addCommands(
      new RightInterpolatorShoot(s_Indexer, s_RightShooterHood, s_RightShooter, s_Feeder, s_Infeed, Degrees.of(2)),
      new LeftInterpolatorShoot(s_Indexer, s_LeftShooterHood, s_LeftShooter, s_Feeder, s_Infeed, Degrees.of(2))
    );

    addRequirements(s_Indexer, s_Infeed, s_RightShooterHood, s_LeftShooterHood, s_RightShooter, s_LeftShooter, s_Feeder);
  }
}
