package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightTurretSS;

public class RightTurretStop extends Command {

  private RightTurretSS s_RightTurret;

  public RightTurretStop(RightTurretSS s_RightTurret) {
    this.s_RightTurret = s_RightTurret;
    addRequirements(s_RightTurret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_RightTurret.Stop();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
