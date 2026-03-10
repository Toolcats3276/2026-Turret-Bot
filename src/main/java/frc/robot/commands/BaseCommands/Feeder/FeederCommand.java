package frc.robot.commands.BaseCommands.Feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSS;

public class FeederCommand extends Command {

  private FeederSS s_Feeder;
  private AngularVelocity speed;


  public FeederCommand(FeederSS s_Feeder, AngularVelocity speed) {
    this.s_Feeder = s_Feeder;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    s_Feeder.setSpeed(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
