package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.subsystems.InfeedSS;

public class InfeedCoCommand extends ParallelCommandGroup {
  public InfeedCoCommand(InfeedSS s_Infeed) {

    addCommands(
      new RepeatCommand(
        new ParallelCommandGroup(
          // new InfeedPID(s_Infeed, Constants.RobotConstants.Infeed.Infeed_POS ,Constants.RobotConstants.Infeed.Max_Speed),
          new InfeedCommand(s_Infeed, .67)
        )
      )
    );

    addRequirements(s_Infeed);
  }
}
