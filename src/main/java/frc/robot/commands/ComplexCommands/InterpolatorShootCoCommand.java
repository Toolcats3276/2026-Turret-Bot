package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.SlavedTurretCommand;
import frc.robot.commands.BaseCommands.Feeder.FeederCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedPivotSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;
import us.hebi.quickbuf.RepeatedString;

public class InterpolatorShootCoCommand extends SequentialCommandGroup {

  public InterpolatorShootCoCommand(IndexerSS s_Indexer,RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret, InfeedSS s_Infeed, InfeedPivotSS s_InfeedPivot, RightShooterHoodSS s_RightShooterHood, LeftShooterHoodSS s_LeftShooterHood, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, FeederSS s_Feeder) {

    addCommands(
      new ParallelCommandGroup(
          new SlavedTurretCommand(s_RightTurret, s_LeftTurret),
          new SequentialCommandGroup(
            new WaitCommand(.75),
            new ParallelCommandGroup(
              new IndexerCommand(s_Indexer, RotationsPerSecond.of(1)),
              new FeederCommand(s_Feeder, RotationsPerSecond.of(1)),
              new InfeedCommand(s_Infeed, RotationsPerSecond.of(.85))
            )
          )
      )
    );
    addRequirements(s_Indexer, s_LeftTurret, s_RightTurret, s_Infeed, s_InfeedPivot, s_RightShooterHood, s_LeftShooterHood, s_RightShooter, s_LeftShooter, s_Feeder);
  }
}
