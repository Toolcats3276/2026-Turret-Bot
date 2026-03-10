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

    if(s_RightTurret.LimeLightTargetBoolean() == true && s_LeftTurret.LimeLightTargetBoolean() == true){
      s_RightTurret.AutoAim(1);
      s_LeftTurret.AutoAim(1);
    }
    else if(s_RightTurret.LimeLightTargetBoolean() == true && s_LeftTurret.LimeLightTargetBoolean() == false){
      s_RightTurret.AutoAim(1);
      s_LeftTurret.PID(s_RightTurret.returnPOS(), 1);
    }
    else if(s_RightTurret.LimeLightTargetBoolean() == false && s_LeftTurret.LimeLightTargetBoolean() == true){
      s_RightTurret.PID(s_LeftTurret.returnPOS(), 1);
      s_LeftTurret.AutoAim(1);
    }
    else {
      s_RightTurret.Stop();
      s_LeftTurret.Stop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
