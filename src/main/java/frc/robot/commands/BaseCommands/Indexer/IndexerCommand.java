package frc.robot.commands.BaseCommands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSS;
import frc.robot.subsystems.IndexerSS;

public class IndexerCommand extends Command {

  private IndexerSS s_Indexer;
  private double speed;


  public IndexerCommand(IndexerSS s_Indexer, double speed) {
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
