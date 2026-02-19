package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Class to wait until double providers base and target are within a margin of error.
 */
public class GenericAwaitBaseTargetWithinError extends Command {
  protected final DoubleSupplier m_base;
  protected final DoubleSupplier m_target;
  private double m_error;

  private boolean m_useStaticTarget = false;
  private double m_staticTarget = 0;

  public GenericAwaitBaseTargetWithinError(DoubleSupplier base, DoubleSupplier target, double error) {
    m_base = base;
    m_target = target;
    m_error = error;
  }

  protected void setUseStaticTarget(boolean value) {
    m_useStaticTarget = value;
  }

  @Override
  public void initialize() {
    m_staticTarget = m_target.getAsDouble();
  }

  protected void setAllowableError(double error) {
    m_error = error;
  }

  protected double getDifferenceFromTarget() {
    return Math.abs(m_base.getAsDouble() - (m_useStaticTarget ? m_staticTarget : m_target.getAsDouble()));
  }

  @Override
  public boolean isFinished() {
    return getDifferenceFromTarget() < m_error;
  }
}
