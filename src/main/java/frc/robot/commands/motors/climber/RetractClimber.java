package frc.robot.commands.motors.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.generics.GenericActiveAwaitMotorPosition;
import frc.robot.subsystems.motors.Climber;

public class RetractClimber extends GenericActiveAwaitMotorPosition {
  public RetractClimber(Climber climber) {
    super(climber, ClimberConstants.Control.RetractedPosition);
  }
}
