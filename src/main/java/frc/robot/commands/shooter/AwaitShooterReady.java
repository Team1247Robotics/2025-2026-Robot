package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import frc.robot.commands.generics.GenericActiveAwaitMotorSpeed;
import frc.robot.subsystems.Shooter;

/**
 * Command that spins shooter to set velocity until reaching velocity. Will finish when the target has been reached.
 */
public class AwaitShooterReady extends GenericActiveAwaitMotorSpeed {
  
  /**
   * Instantiate with a dynamically updating value.
   * @param shooter - The shooter object
   * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
   */
  public AwaitShooterReady(Shooter shooter, DoubleSupplier velocity) {
    super(shooter, velocity);
  }

  /**
   * Instatiate with a statically set value.
   * @param shooter - The shooter object
   * @param velocity - A double representing the target velocity in RPM.
   */
  public AwaitShooterReady(Shooter shooter, double velocity) {
    super(shooter, velocity);
  }
}

