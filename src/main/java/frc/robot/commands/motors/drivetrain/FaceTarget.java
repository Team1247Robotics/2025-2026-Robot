package frc.robot.commands.motors.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Targeting;

public interface FaceTarget {
  interface Pose2d {
    interface Await {
      public class Actively extends FaceHeading.Await.Actively {
        protected static Supplier<Rotation2d> createPoseToRotationFunction(Supplier<edu.wpi.first.math.geometry.Pose2d> poseA, Supplier<edu.wpi.first.math.geometry.Pose2d> poseB) {
          return () -> Targeting.convertFieldRelativeToRobotRelativeTranslation(poseA.get(), poseB.get(), Translation2d.kZero).getAngle();
        }

        protected final DriveSubsystem m_drivetrain;

        /**
         * @param drivetrain - The drivetrain
         * @param targetSupplier - Function that returns a target Pose2d.
         * @param xSupplier - Function that when called returns the target x velocity or effort.
         * @param ySupplier - Function that when called returns the target y velocity or effort.
         */
        public Actively(DriveSubsystem drivetrain, Supplier<edu.wpi.first.math.geometry.Pose2d> targetSupplier, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
          super(drivetrain, createPoseToRotationFunction(drivetrain::getPose, targetSupplier::get), xSupplier, ySupplier);
          m_drivetrain = drivetrain;
        }

        /**
         * @param drivetrain - The drivetrain
         * @param target - Target Pose2d
         * @param xSupplier - Function that when called returns the target x velocity or effort.
         * @param ySupplier - Function that when called returns the target y velocity or effort.
         */
        public Actively(DriveSubsystem drivetrain, edu.wpi.first.math.geometry.Pose2d target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
          this(drivetrain, () -> target, xSupplier, ySupplier);
        }

        /**
         * Protected constructor intended to be used when manually calling {@link #pointToTarget}.
         * @param drivetrain - The drivetrain
         * @param xSupplier - Function that when called returns the target x velocity or effort.
         * @param ySupplier - Function that when called returns the target y velocity or effort.
         */
        protected Actively(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
          this(drivetrain, () -> edu.wpi.first.math.geometry.Pose2d.kZero, xSupplier, ySupplier);
        }

        /**
         * Protected constructor intended to be used when manually calling {@link #pointToTarget(double, double, Pose2d)}
         * @param drivetrain - The drivetrain
         */
        protected Actively(DriveSubsystem drivetrain) {
          this(drivetrain, () -> edu.wpi.first.math.geometry.Pose2d.kZero, () -> 0.0, () -> 0.0);
        }

        /**
         * Updates the target pose of the command.
         * @param newTarget - The new Pose2d.
         */
        public void updateTarget(Supplier<edu.wpi.first.math.geometry.Pose2d> newTarget) {
          m_headingTargetSupplier = createPoseToRotationFunction(m_drivetrain::getPose, newTarget);
        }

        /**
         * Updates the function that is called when updating the target pose.
         * @param newTargetSupplier - The new Pose2d supplier.
         */
        public void updateTarget(edu.wpi.first.math.geometry.Pose2d  newTarget) {
          updateTarget(() -> newTarget);
        }
      }

      public class Passively extends Actively {
        /**
         * @param drivetrain - The drivetrain
         * @param targetSupplier - Function that returns a target Pose2d.
         * @param xSupplier - Function that when called returns the target x velocity or effort.
         * @param ySupplier - Function that when called returns the target y velocity or effort.
         */
        public Passively(DriveSubsystem drivetrain, Supplier<edu.wpi.first.math.geometry.Pose2d> targetSupplier, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
          super(drivetrain, targetSupplier, xSupplier, ySupplier);
        }

        /**
         * @param drivetrain - The drivetrain
         * @param target - Target Pose2d
         * @param xSupplier - Function that when called returns the target x velocity or effort.
         * @param ySupplier - Function that when called returns the target y velocity or effort.
         */
        public Passively(DriveSubsystem drivetrain, edu.wpi.first.math.geometry.Pose2d target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
          super(drivetrain, target, xSupplier, ySupplier);
        }

        /**
         * Protected constructor intended to be used when manually calling {@link #pointToTarget}.
         * @param drivetrain - The drivetrain
         * @param xSupplier - Function that when called returns the target x velocity or effort.
         * @param ySupplier - Function that when called returns the target y velocity or effort.
         */
        protected Passively(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
          super(drivetrain, xSupplier, ySupplier);
        }

        /**
         * Protected constructor intended to be used when manually calling {@link #pointToTarget(double, double, Pose2d)}
         * @param drivetrain - The drivetrain
         */
        protected Passively(DriveSubsystem drivetrain) {
          super(drivetrain);
        }

        @Override
        public void execute() {
          // does nothing
        }
      }
    }

    public class Indefinitely extends Await.Actively {
      /**
       * @param drivetrain - The drivetrain
       * @param targetSupplier - Function that returns a target Pose2d.
       * @param xSupplier - Function that when called returns the target x velocity or effort.
       * @param ySupplier - Function that when called returns the target y velocity or effort.
       */
      public Indefinitely(DriveSubsystem drivetrain, Supplier<edu.wpi.first.math.geometry.Pose2d> targetSupplier, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(drivetrain, targetSupplier, xSupplier, ySupplier);
      }

      /**
       * @param drivetrain - The drivetrain
       * @param target - Target Pose2d
       * @param xSupplier - Function that when called returns the target x velocity or effort.
       * @param ySupplier - Function that when called returns the target y velocity or effort.
       */
      public Indefinitely(DriveSubsystem drivetrain, edu.wpi.first.math.geometry.Pose2d target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(drivetrain, target, xSupplier, ySupplier);
      }

      /**
       * Protected constructor intended to be used when manually calling {@link #pointToTarget}.
       * @param drivetrain - The drivetrain
       * @param xSupplier - Function that when called returns the target x velocity or effort.
       * @param ySupplier - Function that when called returns the target y velocity or effort.
       */
      protected Indefinitely(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(drivetrain, xSupplier, ySupplier);
      }

      /**
       * Protected constructor intended to be used when manually calling {@link #pointToTarget(double, double, Pose2d)}
       * @param drivetrain - The drivetrain
       */
      protected Indefinitely(DriveSubsystem drivetrain) {
        super(drivetrain);
      }
    }
  }
}
