package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.Supplier;

import frc.robot.Constants.HistoryConstants;

public class IntSensorDerivative extends IntSensorHistory {
  protected Supplier<float[]> m_sensorSupplier;
  private ArrayList<IntSensorDerivative> m_dependants = new ArrayList<IntSensorDerivative>();
  public IntSensorDerivative(Supplier<float[]> sensorSupplier) { 
    super(sensorSupplier.get().length - 1 - HistoryConstants.totalArrayEndShift);
    m_sensorSupplier = sensorSupplier;
  }

  public IntSensorDerivative(IntSensorHistory parent) {
    this(parent::getBuffer);
    parent.addDependant(this);
  }

  public int getSyncedFrametime() {
    return HistoryConstants.derivativeWindow + (m_dependants.size() > 0 ? m_dependants.get(0).getSyncedFrametime() : 0);
  }

  @Override
  protected void updateLatest() {
    float[] sensorBuffer = m_sensorSupplier.get();
    m_buffer[m_buffer.length - 1] = (sensorBuffer[sensorBuffer.length - 1] - sensorBuffer[sensorBuffer.length - HistoryConstants.totalArrayEndShift]) / HistoryConstants.totalArrayEndShift;
  }
}
