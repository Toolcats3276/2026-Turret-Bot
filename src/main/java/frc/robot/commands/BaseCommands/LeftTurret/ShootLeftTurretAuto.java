package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftShooterSS;
import frc.robot.subsystems.LeftShooterSS;

public class ShootLeftTurretAuto extends Command {

  private LeftShooterSS s_LeftShooter;


  public ShootLeftTurretAuto(LeftShooterSS s_LeftShooter) {
    this.s_LeftShooter = s_LeftShooter;
  }

  @Override
  public void initialize() {
    s_LeftShooter.setSpeed(s_LeftShooter.ShootPower());
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
