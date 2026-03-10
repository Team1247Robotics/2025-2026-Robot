package frc.robot.subsystems.motors;


import frc.robot.Configs.BeltFeederConfig;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.generics.GenericNeo;


public class BeltFeeder extends GenericNeo {
    public BeltFeeder() {
        super(FeederConstants.BeltFeederCanID, BeltFeederConfig.config);
  }
}



