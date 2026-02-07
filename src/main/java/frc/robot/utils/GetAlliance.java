package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

public class GetAlliance {
  public static boolean isBlueAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return false;

    if (alliance.get().equals(DriverStation.Alliance.Blue)) {
      return true;
    } else {
      return false;
    }
  }

  public static boolean isRedAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return false;
    if (alliance.get().equals(DriverStation.Alliance.Red)) {
      return true;
    } else {
      return false;
    }
  }

  public static boolean isNoAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return true;
    return false;
  }
}
