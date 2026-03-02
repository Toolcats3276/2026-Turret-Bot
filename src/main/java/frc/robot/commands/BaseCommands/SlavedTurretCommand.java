// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BaseCommands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.commands.BaseCommands.RightTurret.RightTurretPIDFollow;
import frc.robot.subsystems.LeftTurretSS;
import frc.robot.subsystems.RightShooterSS;
import frc.robot.subsystems.RightTurretSS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SlavedTurretCommand extends Command {
  /** Creates a new SlavedTurretCommand. */

  private final RightTurretSS s_RightTurret;
  private final LeftTurretSS s_LeftTurret;

  public SlavedTurretCommand(RightTurretSS s_RightTurret, LeftTurretSS s_LeftTurret) {
    this.s_RightTurret = s_RightTurret;
    this.s_LeftTurret = s_LeftTurret;
    addRequirements(s_RightTurret, s_LeftTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putBoolean("Both See it", s_RightTurret.LimeLightTargetBoolean() == true && s_LeftTurret.LimeLightTargetBoolean() == true);
    SmartDashboard.putBoolean("Left Sees it", s_RightTurret.LimeLightTargetBoolean() == false && s_LeftTurret.LimeLightTargetBoolean() == true);
    SmartDashboard.putBoolean("Right Sees it", s_RightTurret.LimeLightTargetBoolean() == true && s_LeftTurret.LimeLightTargetBoolean() == false);

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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
