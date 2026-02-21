package frc.robot.subsystems.motors;

import frc.robot.Configs.ClimberMotor;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.generics.GenericNeo;

public class Climber extends GenericNeo {
  public Climber() {
    super(ClimberConstants.kMotorCanId, ClimberMotor.config);
  }
}
