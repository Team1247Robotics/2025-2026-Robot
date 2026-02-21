package frc.robot.commands.motors.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.generics.GenericSetMotorSpeed;
import frc.robot.subsystems.motors.Intake;

public class RunIntakeIndefinitely extends GenericSetMotorSpeed {
  public RunIntakeIndefinitely(Intake intake) {
    super(intake, IntakeConstants.Control.IntakeSpeed);
  }
}
