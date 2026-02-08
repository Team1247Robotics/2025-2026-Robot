// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 8;

    public static final double kFrontLeftChassisAngularOffset = 0.7006581;
    public static final double kFrontRightChassisAngularOffset = 0.4643519;
    public static final double kBackLeftChassisAngularOffset = 0.2181360;
    public static final double kBackRightChassisAngularOffset = 0.1978697;

    public static final boolean kFrontLeftDriveInverted =  true;
    public static final boolean kFrontRightDriveInverted = true; 
    public static final boolean kBackLeftDriveInverted =   true;
    public static final boolean kBackRightDriveInverted =  true;

    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    //#region TODO: get real measurements for this
    public static final double kTrackWidth = 0.5; // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7; // Distance between front and back wheels on robot
    //#endregion

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
      );

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 0.5; // 4.8;
    public static final double kMaxAngularSpeed = 1; // 2 * Math.PI; // radians per second
  }

  public static final class IndexerConstants {
    public static final int kMotorCanId = 12;

    public static final class Control {
      public static final double allowableError = Math.PI / 8; // Can be off Math.PI / 8 of target to be considered "at target";

      public static final double indexerStdStepSize = Math.PI / 2;

      public static final double stepWaitTime = 0.25; // seconds
    }
  }

  public static final class ShooterConstants {
     public static final int kMotorCanId = 11;

     public static final class Control {
      public static final double allowableError = 10; // RPS (maybe)
     }
  }

  public static final class IndexerSensorConstants {
    public static final int sensorId = 0;
  }

  public static final class ModuleConstants {
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorContants.kVortexFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorReduction = (45.0 * 16.0 * 50.0) / (kDrivingMotorPinionTeeth * 15.0 * 28.0); // L3, see https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.03;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3; // 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;  // 3;
    public static final double kMaxAngularAccelerationRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularAccelerationRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorContants {
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double kVortexFreeSpeedRpm = 6784;
  }

  public static final class PhotonVisionConstants {
    public static final AprilTagFieldLayout kApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Refer to https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // and https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
    public static final Transform3d kRobotToCam0 = new Transform3d(
      new Translation3d(
        0.5,
        0.0,
        0.5
      ),
      new Rotation3d(
        0,
        0,
        0
      )
    );

    // public static final Transform3d kRobotToCam1 = new Transform3d(
    //   new Translation3d(
    //     0.5,
    //     0.0,
    //     0.5
    //   ),
    //   new Rotation3d(
    //     0,
    //     0,
    //     0
    //   )
    // );

    public static final Transform3d[] kRobotToCams = {
      kRobotToCam0,
      // kRobotToCam1,
    };

    public static final String kCamera0Name = "idk";
    // public static final String kCamera1Name = "rio pov";

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
