package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.commands.BaseCommands.SlavedTurretCommand;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretAutoAim;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretPID;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretPIDFollow;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretStop;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretAutoAim;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretPID;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretPIDFollow;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretStop;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightTurretSS;

public class ResetTurretCoCommand extends ParallelCommandGroup {

  public ResetTurretCoCommand(RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret) {

    addCommands(
        new RightTurretPID(s_RightTurret, 0, 1),
        new LeftTurretPID(s_LeftTurret, 0, 1)
      );
      addRequirements(s_RightTurret, s_LeftTurret);
  }
}
