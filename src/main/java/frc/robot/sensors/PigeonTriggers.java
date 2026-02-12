package frc.robot.sensors;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GyroConstants;

public class PigeonTriggers {
  private static final Pigeon2 pigeon = new Pigeon2(21);

  public static boolean isFlat() {
    boolean o =
      pigeon.getAccumGyroX().getValue().abs(Radians) < GyroConstants.flatThreshold.abs(Radians)
      &&
      pigeon.getAccumGyroY().getValue().abs(Radians) < GyroConstants.flatThreshold.abs(Radians)
      &&
      pigeon.getAccumGyroZ().getValue().abs(Radians) < GyroConstants.flatThreshold.abs(Radians);
    SmartDashboard.putBoolean("Robot Is Flat", o);

    return o;
  }

  public static Trigger flat() {
    return new Trigger(PigeonTriggers::isFlat);
  }
}