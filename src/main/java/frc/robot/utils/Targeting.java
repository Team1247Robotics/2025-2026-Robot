package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
// import frc.robot.sensors.LimelightHelpers;

/**
 * Utils to handle conversions between robot and field relative spaces.
 */
public class Targeting {
  /**
   * Place an object in robot relative space into field relative space.
   * @param robotPose - Current pose of the robot in field relative space.
   * @param robotSpaceObject - Pose of the object in robot relative space.
   * @return Pose of object in field relative space.
   */
  public static Pose3d robotRelativeToFieldRelative(Pose2d robotPose, Pose3d robotSpaceObject) {
    Pose3d objectRotated = robotSpaceObject.rotateBy(
      new Rotation3d(
        robotPose.getRotation()
      )
    );
    
    return new Pose3d(
      robotPose.getX() + objectRotated.getX(),
      robotPose.getY() + objectRotated.getY(),
      objectRotated.getZ(),
      objectRotated.getRotation()
    );
  }

  /**
   * Place an object in robot relative space into field relative space.
   * @param robotPose - Current pose of the robot in field relative space.
   * @param robotSpaceObject - Pose of the object in robot relative space.
   * @return Pose of object in field relative space.
   */
  public static Pose2d robotRelativeToFieldRelative(Pose2d robotPose, Pose2d robotSpaceObject) {
    Pose2d objectRotated = robotSpaceObject.rotateBy(
      robotPose.getRotation()
    );

    return new Pose2d(
      robotPose.getX() + objectRotated.getX(),
      robotPose.getY() + objectRotated.getY(),
      objectRotated.getRotation()
    );
  }

  /**
   * Returns a default value if both the X and Y of the base are 0
   * 
   * @param <T> - Expected return type of Supplier.
   * @param base - The value to test of X and Y are 0.
   * @param func - Function to call if base is valid.
   * @param defaultReturn - A default value to return if the base is invalid.
   * @return Return of func or default if invalid.
   */
  public static <T> T defaultIfInvalid(Pose2d base, Supplier<T> func, T defaultReturn) {
    if (base.getX() == 0 && base.getY() == 0 || base.getX() < 0 || base.getY() < 0) {
      return defaultReturn;
    } else {
      return func.get();
    }
  }

  /**
   * Returns a default value if both the X and Y of the base are 0
   * 
   * @param <T> - Expected return type of Supplier.
   * @param base - The value to test of X and Y are 0.
   * @param func - Function to call if base is valid.
   * @param defaultReturn - A default value to return if the base is invalid.
   * @return Return of func or default if invalid.
   */
  public static <T> T defaultIfInvalid(Translation2d base, Supplier<T> func, T defaultReturn) {
    if (base.getX() == 0 && base.getY() == 0) {
      return defaultReturn;
    } else {
      return func.get();
    }
  }

  /**
   * Converts the returns from limelight target tracking into a field relative Pose2d.
   * @param robotPose - Current robot pose in field relative space.
   * @param limelightObject - Pose of the target in robot relative space.
   * @param defaultPose - Default pose to return if the pose is invalid (target not found).
   * @return Field relative target pose.
   */
  public static Pose2d convertLimelightTargetPoseToFieldRelative(Pose2d robotPose, Pose3d limelightObject, Pose2d defaultPose) {
    Pose2d target = new Pose2d(
      limelightObject.getY(),
      limelightObject.getX(),
      limelightObject.getRotation()
          .toRotation2d()
          .minus(robotPose.getRotation())
      );
    return defaultIfInvalid(
      target,
      () -> Targeting.robotRelativeToFieldRelative(robotPose, target),
      defaultPose
    );
  }

  /**
   * Converts Field Relative Translation2d into a robot relative Translation2d where the robot is the origin.
   * 
   * @param robotPose - The position of the robot in field relative space.
   * @param target - The position of the target in field relative space.
   * @param defaultTranslation - Default value to return if the target pose is invalid.
   * @return Robot relative Translation2d or defaultTranslation if target is invalid.
   */
  public static Translation2d convertFieldRelativeToRobotRelativeTranslation(Translation2d robotPose, Translation2d target, Translation2d defaultTranslation) {
    return Targeting.defaultIfInvalid(target, () -> {
      Translation2d targetPoint = target;
      Translation2d out = targetPoint.minus(robotPose);
      return out;
    }, defaultTranslation);
  }

  /**
   * Converts field relative Pose2d into robot relative Translation where the robot is the origin.
   * @param robotPose - Current robot pose in field relative space.
   * @param target - Target pose in field relative space.
   * @param defaultTranslation - Default if target position is invalid.
   * @return Robot relative translation of the target or default if target position is invalid.
   */
  public static Translation2d convertFieldRelativeToRobotRelativeTranslation(Pose2d robotPose, Pose2d target, Translation2d defaultTranslation) {
    return convertFieldRelativeToRobotRelativeTranslation(robotPose.getTranslation(), target.getTranslation(), defaultTranslation);
  }

  // /**
  //  * Gets limelight pose and returns a field relative Pose2d.
  //  * @param robotPose - Current robot pose in field relative space.
  //  * @param defaultPose - Default pose if the value from the limelight is invalid.
  //  * @return Pose2d of target in field relative space.
  //  */
  // public static Pose2d getLimelightTargetAsFieldRelativePose(Pose2d robotPose, Pose2d defaultPose) {
  //     Pose3d tagPosition3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
  //     if (tagPosition3d == null) return defaultPose;

  //     return Targeting.convertLimelightTargetPoseToFieldRelative(robotPose, tagPosition3d, defaultPose);
  // }

  /**
   * Get the current signed angular offset between the A and B headings, accounting for angle wraparound.
   * <p>
   * The result is in the range (-π, π], where a positive value means A must
   * turn counter-clockwise and a negative value means clockwise to reach B.
   *
   * @return Signed angular offset in radians from A to B.
   */
  public double getAngularOffset(Rotation2d rotationA, Rotation2d rotationB) {
    double aHeading = rotationA.getRadians() % (Math.PI * 2);
    double bHeading = rotationB.getRadians() % (Math.PI * 2);

    // Normalize both angles to [0, 2π)
    if (aHeading < 0) aHeading += Math.PI * 2;
    if (bHeading < 0) bHeading += Math.PI * 2;

    // Raw difference
    double offset = bHeading - aHeading;

    // Wrap to (-π, π] — shortest path
    if (offset > Math.PI)  offset -= Math.PI * 2;
    if (offset < -Math.PI) offset += Math.PI * 2;

    return offset;
  }
}
