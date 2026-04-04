package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Feeder.FeederCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedPivotSS;
import frc.robot.subsystems.InfeedSS;

public class OutFeed extends ParallelCommandGroup {

  public OutFeed(InfeedSS s_Infeed, FeederSS s_feeder, IndexerSS s_Indexer) {

    addCommands(
      new RepeatCommand(
        new ParallelCommandGroup(
          new InfeedCommand(s_Infeed, RotationsPerSecond.of(-.89)),
          new FeederCommand(s_feeder, RotationsPerSecond.of(-100)),
          new IndexerCommand(s_Indexer, RotationsPerSecond.of(-100))
        )
      )
    );

    addRequirements(s_Infeed, s_feeder, s_Indexer);
  }
}
