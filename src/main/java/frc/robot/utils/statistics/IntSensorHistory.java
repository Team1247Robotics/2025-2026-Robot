package frc.robot.utils.statistics;

import java.util.ArrayList;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.MedianFilter;

public class IntSensorHistory {
  public static class ArrayTooSmallException extends IllegalArgumentException {
    public ArrayTooSmallException(int actualSize, int minimumSize) {
      super(String.format("Array size %d is too small. Minimum required size is %d.", actualSize, minimumSize));
    }
  }

  protected IntSupplier m_sensorSupplier;
  protected double[] m_buffer;
  protected MedianFilter m_filter = new MedianFilter(15);
  private ArrayList<IntSensorHistory> m_dependants = new ArrayList<IntSensorHistory>();
  private double max = Double.MIN_VALUE;
  private double min = Double.MAX_VALUE;

  public IntSensorHistory(IntSupplier sensorSupplier, int bufferSize) {
    if (bufferSize < 5) { // arbitrary number just to prevent obvious errors
      throw new ArrayTooSmallException(bufferSize, 5);
    }
    
    m_sensorSupplier = sensorSupplier;
    m_buffer = new double[bufferSize];
  }

  protected IntSensorHistory(int bufferSize) {
    m_sensorSupplier = () -> 0;
    m_buffer = new double[bufferSize];
  }

  public void addDependant(IntSensorHistory dependant) {
    m_dependants.add(dependant);
  }

  public int getSyncedFrametime() {
    return (m_dependants.size() > 0 ? m_dependants.get(0).getSyncedFrametime() : 0);
  }

  protected void findNewMax() {
    max = Double.MIN_VALUE;
    for (int i = 0; i < m_buffer.length; i++) {
      double value = m_buffer[i];
      if (value > max) max = value;
    }
  }

  protected void findNewMin() {
    min = Double.MAX_VALUE;
    for (int i = 0; i < m_buffer.length; i++) {
      double value = m_buffer[i];
      if (value < min) min = value;
    }
  }

  protected void shiftBuffer() {
    boolean minMustRecalc = m_buffer[0] == min;
    boolean maxMustRecalc = m_buffer[0] == max;
    for (int i = 0; i < m_buffer.length - 1; i++) {
      m_buffer[i] = m_buffer[i + 1];
    }

    if (minMustRecalc) findNewMin();
    if (maxMustRecalc) findNewMax();
  }

  protected void updateLatest() {
    int sensorValue = m_sensorSupplier.getAsInt() * 10;
    double filtered_value = m_filter.calculate(sensorValue);
    if (filtered_value < min) min = filtered_value;
    if (filtered_value > max) max = filtered_value;
    m_buffer[m_buffer.length - 1] = filtered_value;
  }

  public int getOwnDesyncTime() {
    return 0;
  }

  public double[] getBuffer() {
    return m_buffer;
  }

  public int getLength() {
    return m_buffer.length;
  }

  public double getLastElement() {
    return m_buffer[getLength() - 1];
  }

  /**
   * Gets the latest element that is synced with all children's frametimes. For example, derviatives may take multiple ticks to have a derivative for each value in the top level history buffer. This function will attempt to get the latest element that is calculated for all children.
   * @return
   */
  public double getLastSyncedElement() {
    int index = getLength() - 1 - getSyncedFrametime() + getOwnDesyncTime();
    if (index < 0) {
      throw new ArrayIndexOutOfBoundsException("Your history buffer is not large enough to accomodate the number of or delay created by children.");
    }

    return m_buffer[getLength() - 1 - getSyncedFrametime() + getOwnDesyncTime()];
  }

  /**
   * Starting from the last synced frametime, get the index at that offset. For example, if {@code index} is 0, this function returns the same element as{@link #getLastSyncedElement}, if {@code index} is 1, this function returns the element before {@link #getLastSyncedElement()}.
   * @param index The index offset
   * @return The value held at that index
   */
  public double getElementOffsetFromSync(int index) {
    return m_buffer[getLength() - 1 - getSyncedFrametime() - index + getOwnDesyncTime()];
  }

  public void update() {
    shiftBuffer();
    updateLatest();
    for (int i = 0; i < m_dependants.size(); i++) {
      m_dependants.get(i).update();
    }
  }

  public double getMax() {
    return max;
  }

  public double getMin() {
    return min;
  }
}
