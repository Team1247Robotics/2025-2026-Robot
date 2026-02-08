package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.motors.Indexer;

public class SetIndexerPosition extends Command {
  private final double m_targetPosition;
  protected final Indexer m_indexer;
  protected final double m_allowableError = IndexerConstants.Control.allowableError;
  
  
  public SetIndexerPosition(Indexer indexer, double targetPosition) {
    m_targetPosition = targetPosition;
    m_indexer = indexer;
    addRequirements(indexer);
  }

  protected double getDistanceFromTarget(double target) {
    double currentPos = m_indexer.getPosition();
    return Math.abs(currentPos - target);
  }

  protected double getDistanceFromTarget() {
    return getDistanceFromTarget(m_targetPosition);
  }

  protected void moveToTarget(double target) {
    m_indexer.setPosition(target);
  }

  protected void moveToTarget() {
    moveToTarget(m_targetPosition);
  }

  @Override
  public void execute() {
    moveToTarget();
  }

  @Override
  public boolean isFinished() {
    return getDistanceFromTarget() < m_allowableError;
  }
}
