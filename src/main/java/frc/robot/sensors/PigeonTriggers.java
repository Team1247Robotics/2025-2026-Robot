package frc.robot.sensors;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GyroConstants;

public class PigeonTriggers {
  private static final Pigeon2 pigeon = new Pigeon2(21);

  public static record AccelerationData(double ax, double ay, double az) {};

  // public static record TiltAngles(double pitch, double roll, double theta) {};

  public static Angle calculatePitch(AccelerationData data) {
    double magnitude = Math.sqrt(data.ay * data.ay + data.az * data.az);
    double pitch = Math.atan2(magnitude, data.ax);
    return Radians.of(pitch);
  }

  public static Angle calculateRoll(AccelerationData data) {
    double magnitude = Math.sqrt(data.ax * data.ax + data.az * data.az);
    double roll = Math.atan2(magnitude, data.ay);
    return Radians.of(roll);
  }

  public static Angle calculateTheta(AccelerationData data) {
    double magnitude = Math.sqrt(data.ax * data.ax + data.ay * data.ay);
    double theta = Math.atan2(data.az, magnitude);
    return Radians.of(theta);
  }

  public static Rotation3d calculateAngles() {
    var data = new AccelerationData(
      pigeon.getAccelerationX().getValueAsDouble(),
      pigeon.getAccelerationY().getValueAsDouble(),
      pigeon.getAccelerationZ().getValueAsDouble()
    );
    Angle pitch = calculatePitch(data);
    Angle roll = calculateRoll(data);
    Angle theta = calculateTheta(data);
    return new Rotation3d(roll, pitch, theta);
  }

  public static boolean isFlat() {

    // TODO the code herunder needs to be fixed by calling pigeon.getAccelerationX/Y/Z() and doing the math as explained in the links below
    // https://www.thierry-lequeu.fr/data/AN3461.pdf
		// https://www.analog.com/en/app-notes/an-1057.html

    var angles = calculateAngles();

    boolean o =
      Math.abs(angles.getX()) < GyroConstants.flatThreshold.abs(Radians)
      &&
      Math.abs(angles.getY()) < GyroConstants.flatThreshold.abs(Radians)
    SmartDashboard.putBoolean("Robot Is Flat", o);

    return o;
  }

  public static Trigger flat() {
    return new Trigger(PigeonTriggers::isFlat);
  }
}