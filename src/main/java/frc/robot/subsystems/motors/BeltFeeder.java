package frc.robot.subsystems.motors;


import frc.robot.Configs.BeltFeederConfig;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.generics.GenericVortex;


public class BeltFeeder extends GenericVortex {
    public BeltFeeder() {
        super(FeederConstants.BeltFeederCanID, BeltFeederConfig.config);
  }
}



