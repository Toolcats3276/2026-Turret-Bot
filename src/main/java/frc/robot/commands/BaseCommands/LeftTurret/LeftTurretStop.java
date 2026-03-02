package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftTurretSS;

public class LeftTurretStop extends Command {

  private LeftTurretSS s_LeftTurret;

  public LeftTurretStop(LeftTurretSS s_LeftTurret) {
    this.s_LeftTurret = s_LeftTurret;
    addRequirements(s_LeftTurret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_LeftTurret.Stop();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
