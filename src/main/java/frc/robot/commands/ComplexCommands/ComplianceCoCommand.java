// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Feeder.FeederCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretLinearActuator;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretLinearActuator;
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

public class ComplianceCoCommand extends ParallelCommandGroup {

  public ComplianceCoCommand(RightShooterSS s_RightShooter, RightTurretSS s_RightTurret, LeftShooterSS s_LeftShooter, LeftTurretSS s_LeftTurret, IndexerSS s_Indexer, InfeedSS s_Infeed, InfeedPivotSS s_InfeedPivot, RightShooterHoodSS s_RightShooterHood, LeftShooterHoodSS s_LeftShooterHood, FeederSS s_Feeder) {

    addCommands(
        new FeederCommand(s_Feeder, RotationsPerSecond.of(0)),
        new InfeedCommand(s_Infeed, RotationsPerSecond.of(0)),
        new InfeedPID(s_InfeedPivot, Constants.RobotConstants.Infeed.Infeed_POS, Constants.RobotConstants.Infeed.Max_Speed),
        new ShootLeftTurret(s_LeftShooter, RotationsPerSecond.of(0)),
        new ShootRightTurret(s_RightShooter, 0),
        new IndexerCommand(s_Indexer, RotationsPerSecond.of(0))
    );

      addRequirements(s_Indexer, s_Infeed, s_RightTurret, s_LeftTurret);
  }
}
