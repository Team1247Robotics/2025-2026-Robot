package frc.robot.commands.targeting;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TargetingConstants;

public class targetingCommand {
  private final NetworkTableEntry distEntry =
      NetworkTableInstance.getDefault().getTable("vision").getEntry("distanceMeters");
  private final NetworkTableEntry shooterComputeEntry =
      NetworkTableInstance.getDefault().getTable("").getEntry("ShooterCompute");

  private final InterpolatingDoubleTreeMap rangeToRpmLookup = new InterpolatingDoubleTreeMap();

  private Double m_lastPublishedDistanceMeters = null;
  private double m_lastComputedRpm = ShooterConstants.kTargetSpeed.abs(RPM);

  public targetingCommand() {
    for (double[] sample : TargetingConstants.kRangeToRpmLookup) {
      rangeToRpmLookup.put(sample[0], sample[1]);
    }
  }

  public void BroadcastShotSolution(Pose2d robotPose, int targetId) {
    double distanceMeters = calculateRangeMeters(robotPose, targetId);
    SmartDashboard.putNumber("Distance to Target", distanceMeters);
    if (distanceMeters <= 0) {
      return;
    }

    if (shouldPublishDistance(distanceMeters)) {
      distEntry.setDouble(distanceMeters);
      m_lastPublishedDistanceMeters = distanceMeters;
    }

    double computedRpm = calculateShooterRpm(distanceMeters);
    shooterComputeEntry.setDouble(computedRpm);
    m_lastComputedRpm = computedRpm;
  }

  public double calculateRangeMeters(Pose2d robotPose, int targetId) {
    if (targetId <= 0) {
      return 0;
    }

    var targetPose = PhotonVisionConstants.kApriltagFieldLayout.getTagPose(targetId);
    if (targetPose.isEmpty()) {
      return 0;
    }

    return robotPose.getTranslation().getDistance(targetPose.get().toPose2d().getTranslation());
  }

  public double calculateShooterRpm(double distanceMeters) {
    if (distanceMeters <= 0 || TargetingConstants.kRangeToRpmLookup.length == 0) {
      return m_lastComputedRpm;
    }

    double minRange = TargetingConstants.kRangeToRpmLookup[0][0];
    double maxRange = TargetingConstants.kRangeToRpmLookup[TargetingConstants.kRangeToRpmLookup.length - 1][0];
    double clampedRange = Math.max(minRange, Math.min(maxRange, distanceMeters));
    return rangeToRpmLookup.get(clampedRange);
  }

  public double ConsumeShooterCompute() {
    double computedRpm = shooterComputeEntry.getDouble(m_lastComputedRpm);
    if (computedRpm > 0) {
      m_lastComputedRpm = computedRpm;
    }
    return m_lastComputedRpm*0.9;
  }

  private boolean shouldPublishDistance(double distanceMeters) {
    if (m_lastPublishedDistanceMeters == null) {
      return true;
    }

    return Math.abs(distanceMeters - m_lastPublishedDistanceMeters)
        >= TargetingConstants.kDistancePublishThresholdMeters;
  }
}
