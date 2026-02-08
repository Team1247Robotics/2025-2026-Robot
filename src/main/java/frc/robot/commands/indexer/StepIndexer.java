package frc.robot.commands.indexer;

import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.motors.Indexer;

/**
 * Steps the indexer by the indexer step size constant.
 */
public class StepIndexer extends SetIndexerPosition {
  public StepIndexer(Indexer indexer) {
    super(indexer, indexer.getPosition() + IndexerConstants.Control.indexerStdStepSize);
  }
}
