package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretPID;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretPID;
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
