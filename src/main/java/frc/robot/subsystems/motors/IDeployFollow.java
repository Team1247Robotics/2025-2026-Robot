package frc.robot.subsystems.motors;

import frc.robot.Configs.IDeployFollowConfig;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.generics.GenericNeo;

public class IDeployFollow extends GenericNeo {
  public IDeployFollow() {
    super(IntakeConstants.kDeployFollowMotorCanId, IDeployFollowConfig.config);
  }
}
