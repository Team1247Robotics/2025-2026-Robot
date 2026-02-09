package frc.robot.utils;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TrackerConstants;
import frc.robot.Robot;

public class IntSensorTracker {
  private final IntSupplier m_intSupplier;

  private final IntSensorHistory m_history;
  private final IntSensorDerivative m_historyDerivative;
  private final IntSensorDerivative m_historyDoubleDerivative;

  private double lastHit = Double.MIN_VALUE;

  public IntSensorTracker(IntSupplier intSupplier, int bufferSize) {
    m_intSupplier = intSupplier;
    m_history = new IntSensorHistory(this::getLatestValue, bufferSize);
    m_historyDerivative = new IntSensorDerivative(m_history);
    m_historyDoubleDerivative = new IntSensorDerivative(m_historyDerivative);
  }

  public IntSensorHistory getHistory() {
    return m_history;
  }

  public IntSensorDerivative getDerivative() {
    return m_historyDerivative;
  }

  public IntSensorDerivative getDoubleDerivative() {
    return m_historyDoubleDerivative;
  }

  /**
   * Gets the latest value from the supplier.
   * @return
   */
  public int getLatestValue() {
    int value;
    if (Robot.isSimulation()) {
      value = (int) (Math.cos(Timer.getFPGATimestamp()) * 10) + (int) Math.round(Math.random()) * 1;
    } else {
      value = m_intSupplier.getAsInt();
    }
    return value;
  }

  /**
   * If the derivative is within the allowable error of zero.
   * @return
   */
  public boolean isDerivativeZero() {
    return Math.abs(m_historyDerivative.getLastSyncedElement()) <= TrackerConstants.allowableError;
  }

  /**
   * If the double derivative is positive.
   * @return
   */
  public boolean isConcaveUp() {
    return m_historyDoubleDerivative.getLastSyncedElement() > -TrackerConstants.allowableError / 2;
  }

  /**
   * If the double derivative is negative.
   * @return
   */
  public boolean isConcaveDown() {
    return m_historyDoubleDerivative.getLastSyncedElement() < TrackerConstants.allowableError / 2;
  }

  /**
   * If at a local minimum. This is not debounced.
   * @return
   */
  public boolean atLocalMinimumInstant() {
    boolean zero = isDerivativeZero();
    boolean up = isConcaveUp();
    return zero && up;
  }

  /**
   * If at a local maximum. This is not debounced.
   * @return
   */
  public boolean atLocalMaximumInstant() {
    boolean zero = isDerivativeZero();
    boolean down = isConcaveDown();
    return zero && down;
  }

  private boolean shouldPass() {
    return lastHit < Timer.getFPGATimestamp() - TrackerConstants.debounceTime && isAllowed();
  }

  private boolean isAllowed() {
    return TrackerConstants.settleTime < Timer.getFPGATimestamp();
  }

  private void putDebugs() {
    SmartDashboard.putNumber("Last Hit", lastHit);
    
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());

    SmartDashboard.putBoolean("Debounce OK", shouldPass());

    SmartDashboard.putBoolean("Derivative Zero", isDerivativeZero());
    SmartDashboard.putBoolean("Concave Down", isConcaveDown());
    SmartDashboard.putBoolean("Concave Up", isConcaveUp());
  }

  /**
   * If at a local minimum. Debounced to only fire once per local minimum.
   * @return
   */
  public boolean atLocalMinimum() {
    if (!atLocalMinimumInstant()) return false;
    boolean go = shouldPass();
    lastHit = Timer.getFPGATimestamp();
    return go;
  }

  /**
   * If at a local maximum. Debounced to only fire once per local maximum.
   * @return
   */
  public boolean atLocalMaximum() {
    if (!atLocalMaximumInstant()) return false;
    boolean go = shouldPass();
    lastHit = Timer.getFPGATimestamp();
    return go;
  }

  /**
   * Update all metrics. Must be run every tick for proper functionality.
   */
  public void update() {
    m_history.update();
    putDebugs();
  }
}
