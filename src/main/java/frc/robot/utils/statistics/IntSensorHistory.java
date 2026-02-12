package frc.robot.utils.statistics;

import java.util.ArrayList;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.MedianFilter;

public class IntSensorHistory {
  protected IntSupplier m_sensorSupplier;
  protected double[] m_buffer;
  protected MedianFilter m_filter = new MedianFilter(15);
  private ArrayList<IntSensorHistory> m_dependants = new ArrayList<IntSensorHistory>();
  public IntSensorHistory(IntSupplier sensorSupplier, int bufferSize) {
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

  protected void shiftBuffer() {
    for (int i = 0; i < m_buffer.length - 1; i++) {
      m_buffer[i] = m_buffer[i + 1];
    }
  }

  protected void updateLatest() {
    int sensorValue = m_sensorSupplier.getAsInt() * 10;
    double filtered_value = m_filter.calculate(sensorValue);
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

  public double getLastSyncedElement() {
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

  public double[] asDoubleArray() {
    double[] o = new double[m_buffer.length];
    for (int i = 0; i < m_buffer.length; i++) {
      o[i] = m_buffer[i];
    }
    return o;
  }

  public void update() {
    shiftBuffer();
    updateLatest();
    for (int i = 0; i < m_dependants.size(); i++) {
      m_dependants.get(i).update();
    }
  }
}
