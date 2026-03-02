package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightShooterSS;

public class ShootRightTurretAuto extends Command {

  private RightShooterSS s_RightShooter;


  public ShootRightTurretAuto(RightShooterSS s_RightShooter) {
    this.s_RightShooter = s_RightShooter;
  }

  @Override
  public void initialize() {
    s_RightShooter.setSpeed(s_RightShooter.ShootPower());
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
