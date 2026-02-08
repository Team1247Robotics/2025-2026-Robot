package frc.robot.sensors;

import com.andymark.jni.AM_CAN_Color_Sensor;
import com.andymark.jni.AM_CAN_Color_Sensor.AM_ColorSensorData;

public class CanColorPromixitySensor {
  AM_CAN_Color_Sensor m_sensor;
  public CanColorPromixitySensor(int sensorId) {
    m_sensor = new AM_CAN_Color_Sensor(sensorId);
  }

  protected AM_ColorSensorData getData() {
    return m_sensor.getData();
  }

  public int getBlue() {
    return getData().blue;
  }

  public int getClear() {
    return getData().clearC;
  }

  public int getGreen() {
    return getData().green;
  }

  public int getProximity() {
    return getData().proximity;
  }

  public int getRed() {
    return getData().red;
  }

  public void setLedState(boolean state) {
    if (state) {
      m_sensor.turnLedOn();
    } else {
      m_sensor.turnLedOff();
    }
  }

  public void restartSensor() {
    m_sensor.restartDevice();
  }
}
