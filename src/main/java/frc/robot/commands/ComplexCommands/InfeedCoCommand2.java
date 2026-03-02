package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.subsystems.InfeedPivotSS;
import frc.robot.subsystems.InfeedSS;

public class InfeedCoCommand2 extends ParallelCommandGroup {
  public InfeedCoCommand2(InfeedSS s_Infeed, InfeedPivotSS s_InfeedPivot) {

    addCommands(
      new RepeatCommand(
        new ParallelCommandGroup(
          new InfeedCommand(s_Infeed, .89),
          new InfeedPID(s_InfeedPivot, Constants.RobotConstants.Infeed.Infeed_POS2, Constants.RobotConstants.Infeed.Max_Speed)
        )
      )
    );

    addRequirements(s_Infeed, s_InfeedPivot);
  }
}
