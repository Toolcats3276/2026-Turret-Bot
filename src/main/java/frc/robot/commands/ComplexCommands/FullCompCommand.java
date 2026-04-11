package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.LeftShooterHoodSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterHoodSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;

public class FullCompCommand extends ParallelCommandGroup {
  public FullCompCommand(
      RightShooterSS s_RightShooter,
      RightTurretSS s_RightTurret,
      RightShooterHoodSS s_RightShooterHood,
      LeftShooterSS s_LeftShooter,
      LeftTurretSS s_LeftTurret,
      LeftShooterHoodSS s_LeftShooterHood,
      FeederSS s_Feeder,
      IndexerSS s_Indexer,
      InfeedSS s_Infeed,
      InfeedPivotSS s_InfeedPivot) {

    addCommands(
      // new RepeatCommand(
        new ParallelCommandGroup(
          // new InfeedCommand(s_Infeed, RotationsPerSecond.of(1)),
          new InfeedPID(s_InfeedPivot, Constants.RobotConstants.Infeed.FullComp, Constants.RobotConstants.Infeed.Max_Speed),
          new InstantCommand(() -> s_LeftShooter.Stop()),
          new InstantCommand(() -> s_LeftShooterHood.stowPitch()),
          new InstantCommand(() -> s_RightShooter.Stop()),
          new InstantCommand(() -> s_RightShooterHood.stowPitch()),
          new InstantCommand(() -> s_LeftTurret.stowYaw()),
          new InstantCommand(() -> s_RightTurret.stowYaw()),
          new InfeedCommand(s_Infeed, RotationsPerSecond.of(0)),
          new FeederCommand(s_Feeder, RotationsPerSecond.of(0)),
          new IndexerCommand(s_Indexer, RotationsPerSecond.of(0))
      )
    );

    addRequirements(s_RightShooter, s_Feeder, s_Indexer, s_RightTurret, s_RightShooterHood, s_Infeed, s_LeftShooter, s_LeftShooterHood, s_LeftTurret);
  }
}
