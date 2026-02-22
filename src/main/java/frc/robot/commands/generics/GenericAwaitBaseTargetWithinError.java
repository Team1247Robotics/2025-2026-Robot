package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class to wait until double providers base and target are within a margin of error.
 */
public class GenericAwaitBaseTargetWithinError extends Command {
  protected DoubleSupplier m_base;
  protected DoubleSupplier m_target;
  private double m_error;
  
  private boolean m_useRadianWraparound = false;
  private boolean m_useStaticTarget = false;
  private double m_staticTarget = 0;

  public GenericAwaitBaseTargetWithinError(DoubleSupplier base, DoubleSupplier target, double error) {
    m_base = base;
    m_target = target;
    m_error = error;
    addRequirements(new Subsystem[0]);
  }

  public GenericAwaitBaseTargetWithinError(double base, DoubleSupplier target, double error) {this(() -> base, target, error);}
  public GenericAwaitBaseTargetWithinError(DoubleSupplier base, double target, double error) {this(base, () -> target, error);}

  /**
   * Static target is only accessed once on command initization and that value is used as long as static target is enabled. The value is recorded no matter if static target is enabled, which allows it to be toggled on and off while the command is running.
   * @param value
   */
  protected void setUseStaticTarget(boolean value) {
    m_useStaticTarget = value;
  }

  protected void setUseRadianWraparound(boolean value) {
    m_useRadianWraparound = value;
  }

  @Override
  public void initialize() {
    m_staticTarget = m_target.getAsDouble();
    SmartDashboard.putString("Initilizing", getName());
  }

  protected void setAllowableError(double error) {
    m_error = error;
  }

  protected double getDifferenceFromTarget() {
    double target = m_useStaticTarget ? m_staticTarget : m_target.getAsDouble();
    if (!m_useRadianWraparound) return Math.abs(m_base.getAsDouble() - target);

    SmartDashboard.putNumber("Target", target);
    SmartDashboard.putNumber("Base", m_base.getAsDouble());
    SmartDashboard.putNumber("A", m_base.getAsDouble() - target);
    SmartDashboard.putNumber("B", m_base.getAsDouble() - (Math.PI * 2) - target);
    SmartDashboard.putNumber("C", m_base.getAsDouble() + (Math.PI * 2) - target);

    return Math.min(Math.min(
        Math.abs(m_base.getAsDouble() - target),
        Math.abs(m_base.getAsDouble() - (Math.PI * 2) - target)
      ),
      Math.abs(m_base.getAsDouble() + (Math.PI * 2) - target)
    );
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("difference", getDifferenceFromTarget());
    SmartDashboard.putNumber("error", m_error);
    return getDifferenceFromTarget() < m_error;
  }
}
