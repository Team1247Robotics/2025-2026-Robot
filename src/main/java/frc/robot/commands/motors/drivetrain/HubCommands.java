package frc.robot.commands.motors.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.HubPositions;

public interface HubCommands {
  interface AimAt {
    interface Await {
      class Actively extends FaceTarget.Pose2d.Await.Actively {
        public Actively(SwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
          super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
          m_fieldRelative = fieldRelative;
        }
  
        public Actively(SwerveDrivetrain drivetrain) {
          this(drivetrain, () -> 0, () -> 0, false);
        }
      }
  
      class Passively extends FaceTarget.Pose2d.Await.Passively {
        public Passively(SwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
          super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
          m_fieldRelative = fieldRelative;
          SmartDashboard.putString("aim at hub passively", getRequirements().toString());
        }
  
        public Passively(SwerveDrivetrain drivetrain) {
          this(drivetrain, () -> 0, () -> 0, false);
        }
      }

      static Command Passively(SwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
        return new Passively(drivetrain, xSupplier, ySupplier, fieldRelative);
      }

      static Command Passively(SwerveDrivetrain drivetrain) {
        return new Passively(drivetrain);
      }
    }
  
    class Indefinitely extends FaceTarget.Pose2d.Indefinitely {
      public Indefinitely(SwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
        super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
        m_fieldRelative = fieldRelative;
      }
  
      public Indefinitely(SwerveDrivetrain drivetrain) {
        this(drivetrain, () -> 0, () -> 0, false);
      }
    }

    static Command Indefinitely(SwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
      return new Indefinitely(drivetrain, xSupplier, ySupplier, fieldRelative);
    }

    static Command Indefinitely(SwerveDrivetrain drivetrain) {
      return new Indefinitely(drivetrain);
    }
  }
}
