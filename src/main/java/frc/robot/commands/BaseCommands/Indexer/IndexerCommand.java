package frc.robot.commands.BaseCommands.Indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSS;

public class IndexerCommand extends Command {

  private IndexerSS s_Indexer;
  private AngularVelocity speed;


  public IndexerCommand(IndexerSS s_Indexer, AngularVelocity speed) {
    this.s_Indexer = s_Indexer;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    s_Indexer.setSpeed(speed);
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
