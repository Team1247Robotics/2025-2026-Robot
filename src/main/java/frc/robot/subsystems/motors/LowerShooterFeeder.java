package frc.robot.subsystems.motors;


import frc.robot.Configs.LowerShooterFeederConfig;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.generics.GenericNeo;


public class LowerShooterFeeder extends GenericNeo {
    public LowerShooterFeeder() {
        super(FeederConstants.LowerShooterFeederCanID, LowerShooterFeederConfig.config);
  }
}



