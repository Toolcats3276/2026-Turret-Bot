package frc.robot.commands.BaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightTurretSS;

public class SlavedTurretCommand extends Command {

  private final RightTurretSS s_RightTurret;
  private final LeftTurretSS s_LeftTurret;

  public SlavedTurretCommand(RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret) {
    this.s_RightTurret = s_RightTurret;
    this.s_LeftTurret = s_LeftTurret;
    addRequirements(s_RightTurret, s_LeftTurret);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
