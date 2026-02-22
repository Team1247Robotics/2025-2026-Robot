package frc.robot.commands.motors.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.HubPositions;

public interface HubCommands {
  interface AimAt {
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
          SmartDashboard.putString("aim at hub passively", getRequirements().toString());
        }
  
        public Passively(DriveSubsystem drivetrain) {
          this(drivetrain, () -> 0, () -> 0, false);
        }
      }

      static Command Passively(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
        return new Passively(drivetrain, xSupplier, ySupplier, fieldRelative);
      }

      static Command Passively(DriveSubsystem drivetrain) {
        return new Passively(drivetrain);
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

    static Command Indefinitely(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
      return new Indefinitely(drivetrain, xSupplier, ySupplier, fieldRelative);
    }

    static Command Indefinitely(DriveSubsystem drivetrain) {
      return new Indefinitely(drivetrain);
    }
  }
}
