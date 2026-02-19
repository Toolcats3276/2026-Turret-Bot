package frc.robot.commands.BaseCommands.RightTurret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightShooterSS;

public class ShootRightTurret extends Command {

  private RightShooterSS s_RightShooter;
  private double speed;


  public ShootRightTurret(RightShooterSS s_RightShooter, double speed) {
    this.s_RightShooter = s_RightShooter;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    s_RightShooter.setSpeed(speed);
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
