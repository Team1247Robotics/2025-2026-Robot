package frc.robot.subsystems.motors;

import frc.robot.Configs.FeederMotor;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.generics.GenericNeo;

public class Feeder extends GenericNeo {
  public Feeder() {
    super(FeederConstants.kMotorCanId, FeederMotor.config);
  }
}
