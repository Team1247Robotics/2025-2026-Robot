package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.LimelightHelpers;

public class Targeting {
    public static Rotation2d rotationToTarget(Pose2d robotPose, Pose2d targetPose) {
        Translation2d targetPoint = targetPose.getTranslation();
        return targetPoint.minus(robotPose.getTranslation()).getAngle();
    }

    public static Rotation2d currentRotationOffsetFromTarget(Rotation2d current, Rotation2d goal) {
        return goal.minus(current);
    }

    public static Rotation2d calculateOffsetFromTarget(Pose2d robotPose, Pose2d targetPose) {
        return currentRotationOffsetFromTarget(
            robotPose.getRotation(), rotationToTarget(robotPose, targetPose)
        );
    }

    public static Rotation2d calculateOffsetFromTarget(Pose2d robotPose, Pose3d targetPose3d) {
        Pose2d targetPose = targetPose3d.toPose2d();
        return calculateOffsetFromTarget(robotPose, targetPose);
    }

    public static Pose3d placeRobotSpaceInFieldSpace(Pose2d robotPose, Pose3d robotSpaceObject) {
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

    public static Pose2d placeRobotSpaceInFieldSpace(Pose2d robotPose, Pose2d robotSpaceObject) {
        Pose2d objectRotated = robotSpaceObject.rotateBy(
            robotPose.getRotation()
        );

        return new Pose2d(
            robotPose.getX() + objectRotated.getX(),
            robotPose.getY() + objectRotated.getY(),
            objectRotated.getRotation()
        );
    }

    public static <T> T guardAgainstPoseZero(Pose2d base, Supplier<T> func, T zero) {
        if (base.getX() == 0 && base.getY() == 0) {
            return zero;
        } else {
            return func.get();
        }
    }

    public static Pose2d convertLimelightRobotPoseToFieldPose2d(Pose2d robotPose, Pose3d limelightObject, Pose2d defaultPose) {
        Pose2d target = new Pose2d(
            limelightObject.getY(),
            limelightObject.getX(),
            limelightObject.getRotation()
                .toRotation2d()
                .minus(robotPose.getRotation())
            );
        return guardAgainstPoseZero(
            target,
            () -> Targeting.placeRobotSpaceInFieldSpace(robotPose, target),
            defaultPose
        );
    }

    public static Translation2d convertFieldRelativePoseToRobotRelativeTranslation(Pose2d robotPose, Pose2d target, Translation2d defaultTranslation) {
        return Targeting.guardAgainstPoseZero(target, () -> {
            Translation2d targetPoint = target.getTranslation();
            Translation2d out = targetPoint.minus(robotPose.getTranslation());
            SmartDashboard.putString("target", "X: " + out.getX() + ", Y: " + out.getY() + ", " + out.getAngle().getRadians());
            return out;
        }, defaultTranslation);
    }

    public static Rotation2d convertFieldRelativePoseToRobotRelativeRotation2dHeading(Pose2d robotPose, Pose2d target, Rotation2d defaultRotation) {
        return Targeting.guardAgainstPoseZero(target, () -> {
            Rotation2d targetRotation = Targeting.convertFieldRelativePoseToRobotRelativeTranslation(
                robotPose,
                target,
                Translation2d.kZero
            ).getAngle();
            return targetRotation.minus(robotPose.getRotation());
        }, Rotation2d.kZero);
    }

    public static Pose2d getLimelightTargetRelativePoseAsFieldRelativePose2d(Pose2d robotPose, Pose2d defaultPose) {
        Pose3d tagPosition3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
        if (tagPosition3d == null) return defaultPose;

        return Targeting.convertLimelightRobotPoseToFieldPose2d(robotPose, tagPosition3d, defaultPose);
    }
}
