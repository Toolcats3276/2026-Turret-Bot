package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.LeftTurret.LeftTurretPID;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretPID;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightTurretSS;

public class ResetTurretCoCommand extends ParallelCommandGroup {

  public ResetTurretCoCommand(RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret) {

    addCommands(
        new RightTurretPID(s_RightTurret, 0, Constants.RobotConstants.FrontRightTurret.MAX_SPEED),
        new LeftTurretPID(s_LeftTurret, 0, Constants.RobotConstants.FrontLeftTurret.MAX_SPEED)
      );
      addRequirements(s_RightTurret, s_LeftTurret);
  }
}
