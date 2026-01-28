package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class Controller {
    public static double applyDriveYFilters(DoubleSupplier controllerInput) {
        return -MathUtil.applyDeadband(controllerInput.getAsDouble(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    }

    public static double applyDriveXFilters(DoubleSupplier controllerInput) {
        return MathUtil.applyDeadband(controllerInput.getAsDouble(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    }
}
