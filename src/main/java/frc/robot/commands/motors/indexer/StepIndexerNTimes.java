package frc.robot.commands.motors.indexer;

import frc.robot.commands.bases.DoXNTimes;
import frc.robot.subsystems.Indexer;

public class StepIndexerNTimes extends DoXNTimes {

  public StepIndexerNTimes(Indexer indexer, int n) {
    super(
      n,
      new StepAndPauseIndexer(indexer)
    );
  }
  
}
