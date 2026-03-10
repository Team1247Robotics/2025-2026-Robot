package frc.robot.subsystems.motors;


import frc.robot.Configs.UpperIndexerConfig;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.generics.GenericNeo;


public class UpperIndexer extends GenericNeo {
    public UpperIndexer() {
        super(IndexerConstants.UpperIndexerCanID, UpperIndexerConfig.config);
  }
}



