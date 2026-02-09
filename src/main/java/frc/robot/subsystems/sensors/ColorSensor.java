package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ColorSensorConstants;
import frc.robot.sensors.CanColorPromixitySensor;
import frc.robot.utils.IntSensorDerivative;
import frc.robot.utils.IntSensorHistory;

public class ColorSensor extends SubsystemBase {
  protected CanColorPromixitySensor m_sensor;
  // private int[] m_history = new int[ColorSensorConstants.historyBufferSize];
  private IntSensorHistory m_history = new IntSensorHistory(this::getLatestValue, ColorSensorConstants.historyBufferSize);
  private IntSensorDerivative m_historyDerivative = new IntSensorDerivative(m_history);
  private IntSensorDerivative m_historyDoubleDerivative = new IntSensorDerivative(m_historyDerivative);
  private int count = 0;
  private double lastHit = Double.MIN_VALUE;

  public ColorSensor(int sensorId) {
    m_sensor = new CanColorPromixitySensor(sensorId);
  }

  private int getLatestValue() {
    int value;
    if (Robot.isSimulation()) {
      value = (int) (Math.sin(Timer.getFPGATimestamp()) * 255);
    } else {
      value = m_sensor.getProximity();
    }
    return value;
  }

  private boolean derivativeIsZero() {
    return Math.abs(m_historyDerivative.getLastSyncedElement()) <= ColorSensorConstants.allowableError;
  }

  private boolean isConcaveUp() {
    return m_historyDoubleDerivative.getLastSyncedElement() > 0;
  }

  private boolean isConcaveDown() {
    return m_historyDoubleDerivative.getLastSyncedElement() < 0;
  }

  public boolean atLocalMinimum() {
    return derivativeIsZero() && isConcaveUp();
  }

  public boolean atLocalMaximum() {
    return derivativeIsZero() && isConcaveDown();
  }

  @Override
  public void periodic() {
    m_history.update();
    if (atLocalMinimum()) {
      if (lastHit < Timer.getFPGATimestamp() - 0.5) {
        count++;
      }
      lastHit = Timer.getFPGATimestamp();
    }

    SmartDashboard.putNumber("Color sensor latest", m_history.getLastSyncedElement());
    SmartDashboard.putNumberArray("Color sensor buffer", m_history.asDoubleArray());
    SmartDashboard.putNumber("Color sensor derivative", m_historyDerivative.getLastSyncedElement());
    SmartDashboard.putNumber("Color sensor double derivative", m_historyDoubleDerivative.getLastSyncedElement());
    SmartDashboard.putNumber("color sensor count", count);
  }

  public int getCount() {
    return count;
  }
}
