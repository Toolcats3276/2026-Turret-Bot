// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretLinearActuator;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurret;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedPivotSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;
import frc.robot.vision.LimelightAssistant;

public class ShootCoCommand extends SequentialCommandGroup {


  public ShootCoCommand(IndexerSS s_Indexer, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret, InfeedPivotSS s_InfeedPivot, InfeedSS s_Infeed) {

    addCommands(
      // new ConditionalCommand(
      // //onTrue
      //   new ConditionalCommand(
      //   //ontrue at .375
      //     new ParallelCommandGroup(
      //       new ShootRightTurret(s_RightShooter, .8),
      //       new ShootLeftTurret(s_LeftShooter, .8),

      //       new SequentialCommandGroup(
      //         new WaitCommand(.75),
      //         new IndexerCommand(s_Indexer, 1)
      //       )
      //     ),
      //   //onfalse
      //     new ConditionalCommand(
      //     //ontrue at .27
      //       new ParallelCommandGroup(
      //         new ShootRightTurret(s_RightShooter, .7),
      //         new ShootLeftTurret(s_LeftShooter, .7),

      //         new SequentialCommandGroup(
      //           new WaitCommand(.75),
      //           new IndexerCommand(s_Indexer, 1)
      //         )
      //       ),
      //     //onfalse
      //       new ConditionalCommand(
      //       //ontrue at .15
      //         new ParallelCommandGroup(
      //           new ShootRightTurret(s_RightShooter, .665),
      //           new ShootLeftTurret(s_LeftShooter, .665),

      //           new SequentialCommandGroup(
      //             new WaitCommand(.75),
      //             new IndexerCommand(s_Indexer, 1)
      //           )
      //         ),
      //       //onfalse at .075
      //         new ParallelCommandGroup(
      //           new ShootRightTurret(s_RightShooter, .575),
      //           new ShootLeftTurret(s_LeftShooter, .575),

      //           new SequentialCommandGroup(
      //             new WaitCommand(.75),
      //             new IndexerCommand(s_Indexer, 1)
      //           )
      //         ), 
      //       //condition
      //         () -> s_LeftTurret.LinearActuatorSetPoint() == .15
      //       ), 

      //     //condition
      //       ()-> s_LeftTurret.LinearActuatorSetPoint() == .27
      //     ),

      //   //condition
      //     () -> s_LeftTurret.LinearActuatorSetPoint() == .375
      //   ), 
      // //onFalse
      //   new ConditionalCommand(
          
      //   //ontrue at .375
      //     new ParallelCommandGroup(
      //       new ShootRightTurret(s_RightShooter, .8),
      //       new ShootLeftTurret(s_LeftShooter, .8),

      //       new SequentialCommandGroup(
      //         new WaitCommand(.75),
      //         new IndexerCommand(s_Indexer, 1)
      //       )
      //     ),
      //   //onfalse
      //     new ConditionalCommand(
      //     //ontrue at .27
      //       new ParallelCommandGroup(
      //         new ShootRightTurret(s_RightShooter, .7),
      //         new ShootLeftTurret(s_LeftShooter, .7),

      //         new SequentialCommandGroup(
      //           new WaitCommand(.75),
      //           new IndexerCommand(s_Indexer, 1)
      //         )
      //       ),
      //     //onfalse
      //       new ConditionalCommand(
      //       //ontrue at .15
      //         new ParallelCommandGroup(
      //           new ShootRightTurret(s_RightShooter, .665),
      //           new ShootLeftTurret(s_LeftShooter, .665),

      //           new SequentialCommandGroup(
      //             new WaitCommand(.75),
      //             new IndexerCommand(s_Indexer, 1)
      //           )
      //         ),
      //       //onfalse at .075
      //         new ParallelCommandGroup(
      //           new ShootRightTurret(s_RightShooter, .575),
      //           new ShootLeftTurret(s_LeftShooter, .575),

      //           new SequentialCommandGroup(
      //             new WaitCommand(.75),
      //             new IndexerCommand(s_Indexer, 1)
      //           )
      //         ), 
      //       //condition
      //         () -> s_LeftTurret.LinearActuatorSetPoint() == .15
      //       ), 

      //     //condition
      //       ()-> s_LeftTurret.LinearActuatorSetPoint() == .27
      //     ),

      //   //condition
      //     () -> s_LeftTurret.LinearActuatorSetPoint() == .375
      //   ), 
      // //Condition
      // () -> s_InfeedPivot.returnSetPoint() == .735

      // )
      new ParallelCommandGroup(

        new ShootRightTurret(s_RightShooter, .8),
        new ShootLeftTurret(s_LeftShooter, .8),

        new SequentialCommandGroup(
          new WaitCommand(.75),
          new IndexerCommand(s_Indexer, RotationsPerSecond.of(0))
        )
      )
      );
    addRequirements(s_Indexer, s_LeftShooter, s_RightShooter, s_LeftTurret, s_LeftTurret);
  }
}
