package frc.robot.subsystems.motors;


import frc.robot.Configs.UpperShooterFeederConfig;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.generics.GenericNeo;


public class UpperShooterFeeder extends GenericNeo {
    public UpperShooterFeeder() {
        super(FeederConstants.UpperShooterFeederCanID, UpperShooterFeederConfig.config);
  }
}



