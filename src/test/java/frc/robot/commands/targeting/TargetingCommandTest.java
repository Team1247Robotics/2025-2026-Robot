package frc.robot.commands.targeting;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.ShooterConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TargetingCommandTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void resetNetworkTables() {
    NetworkTableInstance.getDefault().stopServer();
    NetworkTableInstance.getDefault().startLocal();
  }

  @Test
  void broadcastShotSolutionPublishesTargetDistance() {
    targetingCommand command = new targetingCommand();
    NetworkTableEntry distanceEntry =
        NetworkTableInstance.getDefault().getTable("vision").getEntry("distanceMeters");

    var targetPose =
        PhotonVisionConstants.kApriltagFieldLayout
            .getTagPose(AprilTags.BLUE_HUB_RIGHT_RIGHT_TARGET)
            .orElseThrow()
            .toPose2d();
    Pose2d robotPose =
        new Pose2d(
            targetPose.getX() - 2.0,
            targetPose.getY(),
            Rotation2d.kZero);

    command.BroadcastShotSolution(robotPose, AprilTags.BLUE_HUB_RIGHT_RIGHT_TARGET);

    assertEquals(2.0, distanceEntry.getDouble(-1.0), 1e-9);
  }

  @Test
  void calculateShooterRpmInterpolatesAboveDefaultTargetSpeed() {
    targetingCommand command = new targetingCommand();

    double rpm = command.calculateShooterRpm(3.5);

    assertTrue(rpm > ShooterConstants.kTargetSpeed.abs(RPM));
    assertEquals(3475.0, rpm, 1e-9);
  }

  @Test
  void consumeShooterComputeCachesLastPositiveRpm() {
    targetingCommand command = new targetingCommand();
    NetworkTableEntry shooterComputeEntry =
        NetworkTableInstance.getDefault().getTable("").getEntry("ShooterCompute");

    double defaultRpm = ShooterConstants.kTargetSpeed.abs(RPM);
    assertEquals(defaultRpm, command.ConsumeShooterCompute(), 1e-9);

    shooterComputeEntry.setDouble(4321.0);
    assertEquals(4321.0, command.ConsumeShooterCompute(), 1e-9);

    shooterComputeEntry.setDouble(0.0);
    assertEquals(4321.0, command.ConsumeShooterCompute(), 1e-9);
  }
}
