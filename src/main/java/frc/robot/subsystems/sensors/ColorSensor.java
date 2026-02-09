package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorSensorConstants;
import frc.robot.sensors.CanColorPromixitySensor;
import frc.robot.utils.statistics.IntSensorTracker;

public class ColorSensor extends SubsystemBase {
  protected CanColorPromixitySensor m_sensor;
  protected IntSensorTracker m_tracker;
  private int count = 0;

  public ColorSensor(int sensorId) {
    m_sensor = new CanColorPromixitySensor(sensorId);
    m_tracker = new IntSensorTracker(m_sensor::getProximity, ColorSensorConstants.historyBufferSize);
  }

  @Override
  public void periodic() {
    m_tracker.update();
    if (m_tracker.atLocalMinimum()) {
      count++;
    }

    SmartDashboard.putNumber("Color sensor latest", m_tracker.getHistory().getLastSyncedElement());
    SmartDashboard.putNumber("Color sensor derivative", m_tracker.getDerivative().getLastSyncedElement());
    SmartDashboard.putNumber("Color sensor double derivative", m_tracker.getDoubleDerivative().getLastSyncedElement());
    SmartDashboard.putNumber("color sensor count", getCount());
  }

  public int getCount() {
    return count;
  }
}
