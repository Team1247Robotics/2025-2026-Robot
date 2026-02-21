package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.generics.GenericSparkMaxMotor;

/**
 * Command that spins motor to set velocity until reaching position. Will finish when the target has been reached.
 */
public class GenericActiveAwaitMotorPosition extends GenericPassiveAwaitMotorSpeed {
  
  /**
   * Instantiate with a dynamically updating value.
   * @param motor - The shooter object
   * @param target - A double supplier that will return the latest target position.
   */
  public GenericActiveAwaitMotorPosition(GenericSparkMaxMotor motor, DoubleSupplier target) {
    super(motor, target);
    addRequirements(motor);
  }

  /**
   * Instatiate with a statically set value.
   * @param motor - The shooter object
   * @param target - A double representing the target position.
   */
  public GenericActiveAwaitMotorPosition(GenericSparkMaxMotor motor, double target) {
    super(motor, target);
    addRequirements(motor);
  }

  @Override
  public void execute() {
    m_motor.setPosition(m_target.getAsDouble());
  }
}

