// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Feeder.FeederCommand;
import frc.robot.commands.BaseCommands.Indexer.IndexerCommand;
import frc.robot.commands.BaseCommands.LeftTurret.ShootLeftTurret;
import frc.robot.commands.BaseCommands.RightTurret.ShootRightTurret;
import frc.robot.subsystems.FeederSS;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.InfeedPivotSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;

public class ShootCoCommand extends ParallelCommandGroup {


  public ShootCoCommand(IndexerSS s_Indexer, RightShooterSS s_RightShooter, LeftShooterSS s_LeftShooter, RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret, InfeedPivotSS s_InfeedPivot, InfeedSS s_Infeed, FeederSS s_Feeder) {

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
      
      new SequentialCommandGroup(
        new WaitCommand(.75),
        new IndexerCommand(s_Indexer, Constants.RobotConstants.Indexer.MAX_ANGULAR_VELOCITY),
        new FeederCommand(s_Feeder, Constants.RobotConstants.Feeder.MAX_ANGULAR_VELOCITY)
      ),
        new ShootRightTurret(s_RightShooter, .8),
        new ShootLeftTurret(s_LeftShooter, 80)

      );
    addRequirements(s_Indexer, s_LeftShooter, s_RightShooter, s_LeftTurret, s_LeftTurret);
  }
}
