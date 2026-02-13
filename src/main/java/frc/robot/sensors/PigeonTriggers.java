package frc.robot.sensors;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GyroConstants;

public class PigeonTriggers {
  private static final Pigeon2 pigeon = new Pigeon2(21);

  public static boolean isFlat() {

    // TODO the code herunder needs to be fixed by calling pigeon.getAccelerationX/Y/Z() and doing the math as explained in the links below
    // https://www.thierry-lequeu.fr/data/AN3461.pdf
		// https://www.analog.com/en/app-notes/an-1057.html

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