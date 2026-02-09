package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.IntSupplier;

public class IntSensorHistory {
  protected IntSupplier m_sensorSupplier;
  protected float[] m_buffer; // Floats because trying to save memory
  private ArrayList<IntSensorHistory> m_dependants = new ArrayList<IntSensorHistory>();
  public IntSensorHistory(IntSupplier sensorSupplier, int bufferSize) {
    m_sensorSupplier = sensorSupplier;
    m_buffer = new float[bufferSize];
  }

  protected IntSensorHistory(int bufferSize) {
    m_sensorSupplier = () -> 0;
    m_buffer = new float[bufferSize];
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
    int sensorValue = m_sensorSupplier.getAsInt();
    m_buffer[m_buffer.length - 1] = sensorValue;
  }

  public float[] getBuffer() {
    return m_buffer;
  }

  public int getLength() {
    return m_buffer.length;
  }

  public float getLastElement() {
    return m_buffer[getLength() - 1];
  }

  public float getLastSyncedElement() {
    return m_buffer[getLength() - 1 - getSyncedFrametime()];
  }

  public double[] asDoubleArray() {
    double[] o = new double[m_buffer.length];
    for (int i = 0; i < m_buffer.length; i++) {
      o[i] = (float) m_buffer[i];
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
