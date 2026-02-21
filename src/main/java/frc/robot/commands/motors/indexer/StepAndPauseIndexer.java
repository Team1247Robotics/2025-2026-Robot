package frc.robot.commands.motors.indexer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.motors.Indexer;

public class StepAndPauseIndexer extends SequentialCommandGroup {
  public StepAndPauseIndexer(Indexer indexer) {
    addCommands(
      new StepIndexer(indexer),
      Commands.waitSeconds(IndexerConstants.Control.stepWaitTime)
    );
  }
}
