package frc.robot.utils.statistics;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TrackerConstants;
import frc.robot.Robot;

/**
 * Tracks a sensor to track various elements of its behaviour including: running buffer its past states, concavity, and sensors for when hitting a local max or min (without using thresholds).
 */
public class IntSensorTracker {
  private final IntSupplier m_intSupplier;

  private final IntSensorHistory m_history;
  private final IntSensorDerivative m_historyDerivative;
  private final IntSensorDerivative m_historyDoubleDerivative;

  private double lastHit = Double.MIN_VALUE;

  private boolean doDebug = false;

  public IntSensorTracker(IntSupplier intSupplier, int bufferSize) {
    m_intSupplier = intSupplier;
    m_history = new IntSensorHistory(this::getLatestValue, bufferSize);
    m_historyDerivative = new IntSensorDerivative(m_history);
    m_historyDoubleDerivative = new IntSensorDerivative(m_historyDerivative);
  }

  /**
   * Set if values should be sent to SmartDashboard.
   * @param val
   */
  public void setDebug(boolean val) {
    doDebug = val;
  }

  /**
   * Are values being sent to Shuffleboard.
   * @return
   */
  public boolean getDebug() {
    return doDebug;
  }

  /**
   * Get the IntSensorHistory object that records a filtered history of the sensors values.
   * @return
   */
  public IntSensorHistory getHistory() {
    return m_history;
  }

  /**
   * Get the IntSensorDerivative object that records the derivative of the IntSensorHistory object over time.
   * @return
   */
  public IntSensorDerivative getDerivative() {
    return m_historyDerivative;
  }

  /**
   * Get the IntSensorDerivative object that record the derivative of the derivative of the IntSensorHistory object over time (The double derivative of the history).
   * @return
   */
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
    return m_historyDoubleDerivative.getLastSyncedElement() > 0;
  }

  /**
   * If the double derivative is negative.
   * @return
   */
  public boolean isConcaveDown() {
    return m_historyDoubleDerivative.getLastSyncedElement() < 0;
  }

  /**
   * If at a local minimum. This is not debounced or protected from initial boot-up turbulance.
   * @return
   */
  public boolean atLocalMinimumInstant() {
    boolean zero = isDerivativeZero();
    boolean up = isConcaveUp();
    return zero && up;
  }

  /**
   * If at a local maximum. This is not debounced or protected from initial boot-up turbulance.
   * @return
   */
  public boolean atLocalMaximumInstant() {
    boolean zero = isDerivativeZero();
    boolean down = isConcaveDown();
    return zero && down;
  }

  /**
   * Returns true of the last hit was over half a second past. Prevents spamming as a result of noise or integer precision loss.
   * @return If another hit is allowed
   */
  private boolean shouldPass() {
    return lastHit < Timer.getFPGATimestamp() - TrackerConstants.debounceTime && isAllowed();
  }

  /**
   * Returns true after {@link TrackerConstants.settleTime} to eliminate the initial turbulance resulting from arrays being populated with zeros.
   * @return If turbulance has likely settled.
   */
  private boolean isAllowed() {
    return TrackerConstants.settleTime < Timer.getFPGATimestamp();
  }

  /**
   * Sends debug values to SmartDashboard. Does nothing if {@link doDebug} is false.
   */
  private void putDebugs() {
    if (!doDebug) return;
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
