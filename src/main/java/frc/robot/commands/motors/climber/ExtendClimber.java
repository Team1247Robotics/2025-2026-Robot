package frc.robot.commands.motors.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.generics.GenericActiveAwaitMotorPosition;
import frc.robot.subsystems.motors.Climber;

public class ExtendClimber extends GenericActiveAwaitMotorPosition {
  public ExtendClimber(Climber climber) {
    super(climber, ClimberConstants.Control.ExtendedPosition);
  }
}
