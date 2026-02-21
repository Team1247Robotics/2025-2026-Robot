package frc.robot.commands.motors.indexer;

import java.util.function.DoubleSupplier;

import frc.robot.commands.generics.GenericSetMotorPosition;
import frc.robot.subsystems.motors.Indexer;

public class SetIndexerPosition extends GenericSetMotorPosition {
  
  public SetIndexerPosition(Indexer indexer, DoubleSupplier targetSupplier) {
    super(indexer, targetSupplier);
  }
  
  public SetIndexerPosition(Indexer indexer, double targetPosition) {
    super(indexer, () -> targetPosition);
  }
}
