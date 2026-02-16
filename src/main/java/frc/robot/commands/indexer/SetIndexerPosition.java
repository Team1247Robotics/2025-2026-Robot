package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;

public class SetIndexerPosition extends Command {
  private final DoubleSupplier m_targetSupplier;
  protected final Indexer m_indexer;
  protected final double m_allowableError = IndexerConstants.Control.allowableError;
  private double m_target = 0;
  
  public SetIndexerPosition(Indexer indexer, DoubleSupplier targetSupplier) {
    m_targetSupplier = targetSupplier;
    m_indexer = indexer;
    addRequirements(indexer);
  }
  
  public SetIndexerPosition(Indexer indexer, double targetPosition) {
    this(indexer, () -> targetPosition);
  }

  protected double getDistanceFromTarget(double target) {
    double currentPos = m_indexer.getPosition();
    return Math.abs(currentPos - target);
  }

  protected double getDistanceFromTarget() {
    return getDistanceFromTarget(m_target);
  }

  protected void moveToTarget(double target) {
    m_indexer.setPosition(target);
  }

  protected void moveToTarget() {
    moveToTarget(m_target);
  }

  @Override
  public void initialize() {
    m_target = m_targetSupplier.getAsDouble();
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
