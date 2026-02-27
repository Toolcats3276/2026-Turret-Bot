// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretLinearActuator;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurretAuto;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretLinearActuator;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurret;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurretAuto;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;
import frc.robot.vision.LimelightAssistant;

public class ShootCoCommand extends SequentialCommandGroup {

  public ShootCoCommand(IndexerSS s_Indexer, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret) {

    addCommands(
        new ConditionalCommand(
          //ontrue
          new ParallelCommandGroup(
            new ShootRightTurret(s_RightShooter, .7125),
            new ShootLeftTurret(s_LeftShooter, .7125),

            new SequentialCommandGroup(
              new WaitCommand(.35),
              new IndexerCommand(s_Indexer, 1)
            )
          ),
            //onfalse
            new ParallelCommandGroup(
              new ShootRightTurret(s_RightShooter, .6),
              new ShootLeftTurret(s_LeftShooter, .6),

              new SequentialCommandGroup(
                new WaitCommand(.35),
                new IndexerCommand(s_Indexer, 1)
              )
            ),
            //condition
            () -> s_RightTurret.LinearActuatorSetPoint() == .375
          
        )
    );
    addRequirements(s_Indexer, s_LeftShooter, s_RightShooter, s_LeftTurret, s_RightTurret);
  }
}
