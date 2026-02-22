package frc.robot.subsystems.motors;

import frc.robot.Configs.IntakeMotor;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.generics.GenericNeo;

public class Intake extends GenericNeo {
  public Intake() {
    super(IntakeConstants.kMotorCanId, IntakeMotor.config);
  }

  @Override
  public void periodic() {
    sendStatsToDash("Intake");
  }
}