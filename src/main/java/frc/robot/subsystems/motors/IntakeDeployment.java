package frc.robot.subsystems.motors;

import frc.robot.Configs.IntakeDeploymentMotor;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.generics.GenericNeo;

public class IntakeDeployment extends GenericNeo {
  public IntakeDeployment() {
    super(IntakeConstants.kDeploymentMotorCanId, IntakeDeploymentMotor.config);
  }
}
