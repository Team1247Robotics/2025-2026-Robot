package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PhotonVisionConstants;

public class HubPositions {
  private static Pose2d calculateAveragePose2d(Pose2d... pose2ds) {
    double collectiveX = 0;
    double collectiveY = 0;
    for (int i = 0; i < pose2ds.length; i++) {
      Pose2d pose = pose2ds[i];
      collectiveX += pose.getX();
      collectiveY += pose.getY();
    }
    double averageX = collectiveX / pose2ds.length;
    double averageY = collectiveY / pose2ds.length;

    return new Pose2d(averageX, averageY, Rotation2d.kZero);
  }

  private static Pose2d getTag(int id) {
    Optional<Pose3d> tag = PhotonVisionConstants.kApriltagFieldLayout.getTagPose(7);
    if (tag.isEmpty()) {
      throw new Error("Tag not found"); // if this happens on the field we deserved that crash
    }
    return tag.get().toPose2d();
  }

  public static Pose2d Red = calculateAveragePose2d(
    getTag(7), // Left from red outbound
    getTag(6), // Left from red inbound
    getTag(12), // Right from red outbound
    getTag(1) // Right from red inbound
  );

  public static Pose2d Blue = calculateAveragePose2d(
    getTag(17), // Right from blue inbound
    getTag(28), // Right from blue outbound
    getTag(22), // Left from blue inbound
    getTag(23) // Left from blue outbound
  );

  public static Pose2d getAllianceHubPose() {
    if (GetAlliance.isBlueAlliance()) {
      return Blue;
    }

    if (GetAlliance.isRedAlliance()) {
      return Red; // A red spy is in the base
    }

    return new Pose2d(0, 0, Rotation2d.kZero);
  }
}
