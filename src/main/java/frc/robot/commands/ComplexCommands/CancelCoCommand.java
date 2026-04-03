package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Feeder.FeederCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretLinearActuator;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretPID;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretLinearActuator;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretPID;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurret;
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

public class CancelCoCommand extends ParallelCommandGroup {

  public CancelCoCommand(IndexerSS s_Indexer, InfeedPivotSS s_InfeedPivot, RightShooterHoodSS s_RightShooterHood, LeftShooterHoodSS s_LeftShooterHood, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, LeftTurretSS s_LeftTurret, RightTurretSS s_RightTurret, InfeedSS s_Infeed, FeederSS s_feeder) {

    addCommands(
      new RepeatCommand(
        new ParallelCommandGroup(
          new InfeedCommand(s_Infeed, RotationsPerSecond.of(0)),
          new FeederCommand(s_feeder, RotationsPerSecond.of(0)),
          new IndexerCommand(s_Indexer, RotationsPerSecond.of(0)),
          new ShootLeftTurret(s_LeftShooter, 0),
          new ShootRightTurret(s_RightShooter, 0),
          // new LeftTurretPID(s_LeftTurret, 0, Constants.RobotConstants.FrontLeftTurret.MAX_SPEED),
          // new RightTurretPID(s_RightTurret, 0, Constants.RobotConstants.FrontRightTurret.MAX_SPEED),
          new RightTurretLinearActuator(s_RightShooterHood, Millimeter.of(0)),
          new LeftTurretLinearActuator(s_LeftShooterHood, Millimeter.of(0))
        )
      )
    );

    addRequirements(s_Infeed, s_feeder, s_Indexer);
  }
}
