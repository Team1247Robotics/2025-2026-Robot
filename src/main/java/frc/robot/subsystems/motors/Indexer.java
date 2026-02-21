package frc.robot.subsystems.motors;

import frc.robot.Configs.IndexerMotor;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.generics.GenericNeo;

public class Indexer extends GenericNeo {
  public Indexer() {
    super(IndexerConstants.kMotorCanId, IndexerMotor.config);
  }
}
