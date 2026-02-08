package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/**
 * Command that spins shooter to set velocity until reaching velocity. Intended to be used in a command sequence to block until the shooter is ready.
 */
public class SpinUpShooter extends Command {
  private DoubleSupplier m_velocity;
  private Shooter m_shooter;
  protected final double m_allowableError = ShooterConstants.Control.allowableError;
  public SpinUpShooter(Shooter shooter, DoubleSupplier velocity) {
    m_velocity = velocity;
    m_shooter = shooter;
  }

  private double rpmDifferenceToTarget() {
    return Math.abs(m_shooter.getVelocity() - m_velocity.getAsDouble()); // this is actually in RPM, not RPS, because the encoder returns RPM.
  }

  @Override
  public void execute() {
    m_shooter.setVelocity(m_velocity.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return rpmDifferenceToTarget() < m_allowableError;
  }
}

