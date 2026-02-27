// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurret;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;

public class ComplianceCoCommand extends ParallelCommandGroup {

  public ComplianceCoCommand(RightShooterSS s_RightShooter, RightTurretSS s_RightTurret, LeftShooterSS s_LeftShooter, LeftTurretSS s_LeftTurret, IndexerSS s_Indexer, InfeedSS s_Infeed) {

    addCommands(
        // new RightTurretPID(s_RightTurret, .23, .75),
        // new LeftTurretPID(s_LeftTurret, .12, .75)
        new InfeedCommand(s_Infeed, 0),
        new ShootRightTurret(s_RightShooter, 0),
        new ShootLeftTurret(s_LeftShooter, 0),
        new IndexerCommand(s_Indexer, 0)
    );

      addRequirements(s_Indexer, s_Infeed, s_RightShooter, s_LeftShooter, s_RightTurret, s_LeftTurret);
  }
}
