package frc.robot.utils.statistics;

import java.util.ArrayList;
import java.util.function.Supplier;

import frc.robot.Constants.TrackerConstants;

public class IntSensorDerivative extends IntSensorHistory {
  protected Supplier<double[]> m_sensorSupplier;

  private ArrayList<IntSensorDerivative> m_dependants = new ArrayList<IntSensorDerivative>();
  private final int m_endShift;

  public IntSensorDerivative(Supplier<double[]> sensorSupplier, short endShift) { 
    super(sensorSupplier.get().length - 1 - endShift);
    m_sensorSupplier = sensorSupplier;
    m_endShift = endShift;
  }

  public IntSensorDerivative(Supplier<double[]> sensorSupplier, int window) {
    this(sensorSupplier, (short) TrackerConstants.calculateEndShift(window));
  }

  public IntSensorDerivative(Supplier<double[]> sensorSupplier) {
    this(sensorSupplier, TrackerConstants.calculateEndShift(TrackerConstants.derivativeWindow));
  }

  public IntSensorDerivative(IntSensorHistory parent) {
    this(parent::getBuffer);
    parent.addDependant(this);
  }

  public IntSensorDerivative(IntSensorHistory parent, int window) {
    this(parent::getBuffer, window);
    parent.addDependant(this);
  }

  public int getSyncedFrametime() {
    return ((m_endShift - 1) / 2) + (m_dependants.size() > 0 ? m_dependants.get(0).getSyncedFrametime() : 0);
  }

  @Override
  protected void updateLatest() {
    double[] sensorBuffer = m_sensorSupplier.get();
    double change = (sensorBuffer[sensorBuffer.length - 1] - sensorBuffer[sensorBuffer.length - 1 - m_endShift]) / (m_endShift - 2);
    // float filteredChange = (float) m_filter.calculate(change);
    m_buffer[m_buffer.length - 1] = change;
  }
}
