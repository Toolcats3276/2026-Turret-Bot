package frc.robot.commands.BaseCommands.LeftTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftShooterSS;

public class ShootLeftTurret extends Command {

  private LeftShooterSS s_LeftShooter;
  private double speed;

  public ShootLeftTurret(LeftShooterSS s_LeftShooter, double speed) {
    this.s_LeftShooter = s_LeftShooter;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    s_LeftShooter.setSpeed(speed);
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
