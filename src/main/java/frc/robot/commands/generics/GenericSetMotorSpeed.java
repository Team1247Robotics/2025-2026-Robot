package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.generics.GenericSparkMaxMotor;

/**
 * Command that spins shooter to set velocity indefinitely.
 */
public class GenericSetMotorSpeed extends GenericActiveAwaitMotorSpeed {
  /**
   * Instantiate with a dynamically updating value.
   * @param motor - The shooter object
   * @param target - A double supplier that will return the latest target velocity in RPM every tick.
   */
  public GenericSetMotorSpeed(GenericSparkMaxMotor motor, DoubleSupplier target) {
    super(motor, target);
  }

  /**
   * Instatiate with a statically set value.
   * @param motor - The shooter object
   * @param target - A double representing the target velocity in RPM.
   */
  public GenericSetMotorSpeed(GenericSparkMaxMotor motor, double target) {
    super(motor, target);
  }

  @Override
  public void execute() {
    m_motor.setVelocity(m_target.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}