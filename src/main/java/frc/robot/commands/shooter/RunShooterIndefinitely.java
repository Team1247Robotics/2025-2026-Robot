package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import frc.robot.commands.generics.GenericSetMotorSpeed;
import frc.robot.subsystems.Shooter;

/**
 * Indefinitely holds the speed of the shooter until forcefully interrupted. Intended to be used after {@link AwaitShooterReady} in parallel with indexer and intake.
 */
public class RunShooterIndefinitely extends GenericSetMotorSpeed {
  /**
   * Instantiate with a dynamically updating value.
   * @param shooter - The shooter object
   * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
   */
  public RunShooterIndefinitely(Shooter shooter, DoubleSupplier velocity) {
    super(shooter, velocity);
  }

  /**
   * Instatiate with a statically set value.
   * @param shooter - The shooter object
   * @param velocity - A double representing the target velocity in RPM.
   */
  public RunShooterIndefinitely(Shooter shooter, double velocity) {
    super(shooter, velocity);
  }
}
