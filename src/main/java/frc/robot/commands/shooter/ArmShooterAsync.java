package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Shooter;

/**
 * Indefinitely holds the speed of the shooter until forcefully interrupted. Intended to be used after {@link ArmShooterBlocking} in parallel with indexer and intake.
 */
public class ArmShooterAsync extends ArmShooterBlocking {
  public ArmShooterAsync(Shooter shooter, DoubleSupplier velocity) {
    super(shooter, velocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
