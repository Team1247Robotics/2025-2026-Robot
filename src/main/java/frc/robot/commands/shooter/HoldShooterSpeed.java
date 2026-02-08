package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Shooter;

/**
 * Indefinitely holds the speed of the shooter until forcefully interupted. Intended to be used after {@link SpinUpShooter} in parallel with indexer and intake.
 */
public class HoldShooterSpeed extends SpinUpShooter {
  public HoldShooterSpeed(Shooter shooter, DoubleSupplier velocity) {
    super(shooter, velocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
