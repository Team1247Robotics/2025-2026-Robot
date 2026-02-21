package frc.robot.commands.motors.drivetrain;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.HubPositions;

public interface FaceHub {
  interface Await {
    class Actively extends FaceTarget.Pose2d.Await.Actively {
      public Actively(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
        super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
        m_fieldRelative = fieldRelative;
      }

      public Actively(DriveSubsystem drivetrain) {
        this(drivetrain, () -> 0, () -> 0, false);
      }
    }

    class Passively extends FaceTarget.Pose2d.Await.Passively {
      public Passively(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
        super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
        m_fieldRelative = fieldRelative;
      }

      public Passively(DriveSubsystem drivetrain) {
        this(drivetrain, () -> 0, () -> 0, false);
      }
    }
  }

  class Indefinitely extends FaceTarget.Pose2d.Indefinitely {
    public Indefinitely(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
      super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
      m_fieldRelative = fieldRelative;
    }

    public Indefinitely(DriveSubsystem drivetrain) {
      this(drivetrain, () -> 0, () -> 0, false);
    }
  }
}
