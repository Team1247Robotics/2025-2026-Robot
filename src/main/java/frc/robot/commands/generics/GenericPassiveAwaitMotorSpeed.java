package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.GenericConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

/**
 * A command that does nothing and will only complete when the given motor reaches the given speed.
 * @implNote Motor can be accessed under {@link #m_motor}, target can be accessed under {@link #m_target}.
 */
public class GenericPassiveAwaitMotorSpeed extends GenericAwaitBaseTargetWithinError {
  protected final GenericSparkMaxMotor m_motor;
  public GenericPassiveAwaitMotorSpeed(GenericSparkMaxMotor motor, DoubleSupplier targetSupplier) {
    super(motor::getVelocity, targetSupplier, GenericConstants.MotorVelocityControlAllowableError);
    m_motor = motor;
  }

  public GenericPassiveAwaitMotorSpeed(GenericSparkMaxMotor motor, double target) {
    this(motor, () -> target);
  }
}
