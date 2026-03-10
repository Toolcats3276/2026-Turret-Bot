package frc.robot.commands.BaseCommands.Infeed;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfeedSS;

public class InfeedCommand extends Command {

  private InfeedSS s_Infeed;
  private AngularVelocity speed;


  public InfeedCommand(InfeedSS s_Infeed, AngularVelocity speed) {
    this.s_Infeed = s_Infeed;
    this.speed = speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_Infeed.SetSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
