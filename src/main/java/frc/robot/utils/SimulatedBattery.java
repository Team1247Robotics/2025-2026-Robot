package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedBattery extends SubsystemBase {
  private double m_Voltage = RobotController.getBatteryVoltage();

  private ArrayList<DoubleSupplier> m_Suppliers = new ArrayList<DoubleSupplier>();

  public double getVoltage() {
    return m_Voltage;
  }

  public void registerPowerDrain(DoubleSupplier supplier) {
    m_Suppliers.add(supplier);
  }

  @Override
  public void simulationPeriodic() {
    Supplier<double[]> currents = (() -> {
      double[] current = new double[m_Suppliers.size()];
      for (int i = 0; i < m_Suppliers.size(); i++) {
        current[i] = (m_Suppliers.get(i).getAsDouble());
      }
      return current;
    });
    m_Voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(currents.get());
    RoboRioSim.setVInVoltage(m_Voltage);
  }
}
