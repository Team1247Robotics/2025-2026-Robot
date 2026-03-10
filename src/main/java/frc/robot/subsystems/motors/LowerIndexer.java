package frc.robot.subsystems.motors;


import frc.robot.Configs.LowerIndexerConfig;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.generics.GenericVortex;


public class LowerIndexer extends GenericVortex {
    public LowerIndexer() {
        super(IndexerConstants.LowerIndexerCanID, LowerIndexerConfig.config);
  }
}



