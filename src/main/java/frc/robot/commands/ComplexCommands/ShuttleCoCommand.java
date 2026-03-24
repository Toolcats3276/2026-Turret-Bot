package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Feeder.FeederCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
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

public class ShuttleCoCommand extends SequentialCommandGroup {

  public ShuttleCoCommand(IndexerSS s_Indexer,RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret, InfeedSS s_Infeed, InfeedPivotSS s_InfeedPivot, RightShooterHoodSS s_RightShooterHood, LeftShooterHoodSS s_LeftShooterHood, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, FeederSS s_Feeder) {

    addCommands(
      new ParallelCommandGroup(
          new SequentialCommandGroup(
            new WaitCommand(1.5),
            new ParallelCommandGroup(
              new IndexerCommand(s_Indexer, RotationsPerSecond.of(1)),
              new FeederCommand(s_Feeder, RotationsPerSecond.of(1))
            )
          ),
            new ShootRightTurret(s_RightShooter, 85),
            new ShootLeftTurret(s_LeftShooter, RotationsPerSecond.of(85)),
            new RightTurretPID(s_RightTurret, 0, Constants.RobotConstants.FrontRightTurret.MAX_SPEED),
            new LeftTurretPID(s_LeftTurret, 0, Constants.RobotConstants.FrontLeftTurret.MAX_SPEED),
            new LeftTurretLinearActuator(s_LeftShooterHood, Millimeter.of(.75)),
            new RightTurretLinearActuator(s_RightShooterHood, Millimeter.of(.75)),
            new InfeedCoCommand(s_Infeed, s_InfeedPivot)
      )
    );
  }
}
