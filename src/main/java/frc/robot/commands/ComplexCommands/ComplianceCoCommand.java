// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretAutoAim;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretLinearActuator;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretPID;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretLinearActuator;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretPID;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurret;
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

  public ComplianceCoCommand(RightShooterSS s_RightShooter, RightTurretSS s_RightTurret, LeftShooterSS s_LeftShooter, LeftTurretSS s_LeftTurret, IndexerSS s_Indexer, InfeedSS s_Infeed, InfeedPivotSS s_InfeedPivot, RightShooterHoodSS s_RightShooterHood, LeftShooterHoodSS s_LeftShooterHood) {

    addCommands(
        new RightTurretLinearActuator(s_RightShooterHood, .075),
        new LeftTurretLinearActuator(s_LeftShooterHood, Millimeter.of(.1)),
        new InfeedCommand(s_Infeed, 0),
        new InfeedPID(s_InfeedPivot, Constants.RobotConstants.Infeed.Infeed_POS, Constants.RobotConstants.Infeed.Max_Speed),
        new ShootLeftTurret(s_LeftShooter, 0),
        new ShootRightTurret(s_RightShooter, 0),
        new IndexerCommand(s_Indexer, RotationsPerSecond.of(0))
    );

      addRequirements(s_Indexer, s_Infeed, s_RightTurret, s_LeftTurret);
  }
}
