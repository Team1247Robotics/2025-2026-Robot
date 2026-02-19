package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

/**
 * A command that does nothing and will only complete when the given motor reaches the given speed.
 * @implNote Only should be necessary to set the target speed. All other parts can be left as-is.
 * @implNote Use {@link #setAllowableError(double)} to overwrite default allowable error.
 * @implNote Motor can be accessed under {@link #m_motor}, target can be accessed under {@link #m_target}.
 */
public class GenericPassiveAwaitMotorSpeed extends Command {
  protected final GenericSparkMaxMotor m_motor;
  protected final DoubleSupplier m_targetSupplier;
  protected double m_allowableError = GenericConstants.MotorVelocityControlAllowableError;

  public GenericPassiveAwaitMotorSpeed(GenericSparkMaxMotor motor, DoubleSupplier targetSupplier) {
    m_motor = motor;
    m_targetSupplier = targetSupplier;
    addRequirements(m_motor);
  }

  public GenericPassiveAwaitMotorSpeed(GenericSparkMaxMotor motor, double target) {
    this(motor, () -> target);
  }

  protected void setAllowableError(double error) {
    m_allowableError = error;
  }

  protected double getDifferenceFromTarget() {
    return Math.abs(m_motor.getVelocity() - m_targetSupplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return getDifferenceFromTarget() < m_allowableError;
  }
}
