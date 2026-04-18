package frc.robot.commands.ComplexCommands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.commands.BaseCommands.Infeed.InfeedCommand;
import frc.robot.commands.BaseCommands.Infeed.InfeedPID;
import frc.robot.subsystems.InfeedPivotSS;
import frc.robot.subsystems.InfeedSS;

public class InfeedCoCommand extends ParallelCommandGroup {
  public InfeedCoCommand(InfeedPivotSS s_InfeedPivot) {

    addCommands(
      // new RepeatCommand(
        new ParallelCommandGroup(
          // new InfeedCommand(s_Infeed, RotationsPerSecond.of(1)),
          new InfeedPID(s_InfeedPivot, Constants.RobotConstants.Infeed.Infeed_POS, Constants.RobotConstants.Infeed.Max_Speed)
        // )
      )
    );

    addRequirements(s_InfeedPivot);
  }
}
