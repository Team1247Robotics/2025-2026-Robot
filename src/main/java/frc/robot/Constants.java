// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.LedStrip.LedStripConfig;
import frc.robot.subsystems.motors.SDSSwerveModule.SDSSwerveModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum Feature {
    Shooter,
    Indexer,
    Feeder,
    Climber,
    Intake,
    IntakeDeployment
  }

  public static boolean isFeatureEnabled(Feature[] features, Feature featureToTest) {
    for (int i = 0; i < features.length; i++) {
      var feat = features[i];
      if (feat.equals(featureToTest)) return true;
    }
    return false;
  }

  public static boolean isFeatureEnabled(Feature[] features, Feature... featuresToTest) {
    for (int i = 0; i < featuresToTest.length; i++) {
      if (!isFeatureEnabled(features, featuresToTest[i])) return false;
    }
    return true;
  }

  public static final boolean UseTestBot = true;

  public static final class GenericConstants {
    public static final Angle kMotorPositionControlAllowableError = Radians.of(Math.PI / 2);
    public static final AngularVelocity kMotorVelocityControlAllowableError = RPM.of(100);
  }

  public static final class DriveConstants {
    public static enum ControllerType { kSparkMax, kSparkFlex }

    /** Wheel configs specific for the main robot */
    public static final class MainBotConfigs {
      public static final ControllerType DriveType = ControllerType.kSparkMax;
      public static final ControllerType TurningType = ControllerType.kSparkMax;

      public static final boolean kFrontLeftDriveInverted =  false;
      public static final boolean kFrontRightDriveInverted = false; 
      public static final boolean kBackLeftDriveInverted =   false;
      public static final boolean kBackRightDriveInverted =  false;

      public static final Angle kFrontLeftChassisAngularOffset  = Rotations.of(0.7789029);
      public static final Angle kFrontRightChassisAngularOffset = Rotations.of(0.3413826);
      public static final Angle kBackLeftChassisAngularOffset   = Rotations.of(0.9801169);
      public static final Angle kBackRightChassisAngularOffset  = Rotations.of(0.3838108);

      public static final SDSSwerveModuleConfig frontLeftSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          kFrontLeftChassisAngularOffset,
          kFrontLeftDriveInverted
        );
      public static final SDSSwerveModuleConfig rearLeftSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          kBackLeftChassisAngularOffset,
          kBackLeftDriveInverted
        );

      public static final SDSSwerveModuleConfig frontRightSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          kFrontRightChassisAngularOffset,
          kFrontRightDriveInverted
        );

      public static final SDSSwerveModuleConfig rearRightSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          kBackRightChassisAngularOffset,
          kBackRightDriveInverted
        );
    }

    /** Wheel configs specific for the test swerve bot */
    public static final class TestBotConfigs {
      public static final ControllerType DriveType = ControllerType.kSparkMax;
      public static final ControllerType TurningType = ControllerType.kSparkMax;

      public static final boolean kFrontLeftDriveInverted  = !false;
      public static final boolean kFrontRightDriveInverted = !false; 
      public static final boolean kBackLeftDriveInverted   = !false;
      public static final boolean kBackRightDriveInverted  = !false;

      public static final Angle kFrontLeftChassisAngularOffset  = Rotations.of(0.7006581);
      public static final Angle kFrontRightChassisAngularOffset = Rotations.of(0.4643519);
      public static final Angle kBackLeftChassisAngularOffset   = Rotations.of(0.2181360);
      public static final Angle kBackRightChassisAngularOffset  = Rotations.of(0.1978697);

      public static final SDSSwerveModuleConfig frontLeftSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          kFrontLeftChassisAngularOffset,
          kFrontLeftDriveInverted
        );
      public static final SDSSwerveModuleConfig rearLeftSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          kBackLeftChassisAngularOffset,
          kBackLeftDriveInverted
        );

      public static final SDSSwerveModuleConfig frontRightSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          kFrontRightChassisAngularOffset,
          kFrontRightDriveInverted
        );

      public static final SDSSwerveModuleConfig rearRightSwerveConfig = new SDSSwerveModuleConfig(
          DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          kBackRightChassisAngularOffset,
          kBackRightDriveInverted
        );
    }

    public static final ControllerType TurningControllerType = UseTestBot ? TestBotConfigs.TurningType : MainBotConfigs.TurningType;
    public static final ControllerType DriveControllerType   = UseTestBot ? TestBotConfigs.DriveType   : MainBotConfigs.DriveType;

    public static SparkBase createMotorController(ControllerType type, int canId) {
      switch (type) {
        case kSparkFlex:
          return new SparkFlex(canId, MotorType.kBrushless);
        case kSparkMax:
          return new SparkMax(canId, MotorType.kBrushless);
        default:
          return null;
      }
    }

    //#region Configs
    public static final SDSSwerveModuleConfig frontLeftConfig  = UseTestBot ? TestBotConfigs.frontLeftSwerveConfig  : MainBotConfigs.frontLeftSwerveConfig;
    public static final SDSSwerveModuleConfig frontRightConfig = UseTestBot ? TestBotConfigs.frontRightSwerveConfig : MainBotConfigs.frontRightSwerveConfig;
    public static final SDSSwerveModuleConfig rearLeftConfig   = UseTestBot ? TestBotConfigs.rearLeftSwerveConfig   : MainBotConfigs.rearLeftSwerveConfig;
    public static final SDSSwerveModuleConfig rearRightConfig  = UseTestBot ? TestBotConfigs.rearRightSwerveConfig  : MainBotConfigs.rearRightSwerveConfig;
    //#endregion

    //#region CAN Ids
    public static final int kFrontLeftDriveMotorPort  = 5;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort   = 7;
    public static final int kBackRightDriveMotorPort  = 1;

    public static final int kFrontLeftTurningMotorPort  = 6;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort   = 8;
    public static final int kBackRightTurningMotorPort  = 2;
    //#endregion

    public static final Time kDrivePeriod = Seconds.of(TimedRobot.kDefaultPeriod);

    public static final Distance kTrackWidth = Inches.of(24.5); //Inches.of(22); // Distance between centers of right and left wheels on robot - Was .5
    public static final Distance kWheelBase  = Inches.of(18.5); //Inches.of(21.875); // Distance between front and back wheels on robot - Was .7
  
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase.div(2), kTrackWidth.div(2)),
        new Translation2d(kWheelBase.div(2), kTrackWidth.div(2).unaryMinus()),
        new Translation2d(kWheelBase.div(2).unaryMinus(), kTrackWidth.div(2)),
        new Translation2d(kWheelBase.div(2).unaryMinus(), kTrackWidth.div(2).unaryMinus())
      );

    public static final boolean kGyroReversed = false;

    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.8);
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(2 * Math.PI);
  }

  public static final class FeederConstants {
    public static final int kMotorCanId = 13;

    public static final class Control {
      public static final AngularVelocity kTargetSpeed = RPM.of(1000);
    }
  }

  public static final class ClimberConstants {
    public static final int kMotorCanId = 14;

    public static final class Control {
      public static final Angle kRetractedPosition = Rotations.of(100);
      public static final Angle kExtendedPosition  = Rotations.zero();
    }
  }

  public static final class IntakeConstants {
    public static final int kMotorCanId = 10;
    public static final int kDeploymentMotorCanId = 15;
    public static final AngularVelocity kMotorFreeSpeed = RPM.of(4600);

    public static final class Control {
      public static final AngularVelocity kIntakeSpeed   = RPM.of(2000);
      public static final AngularVelocity kDispenseSpeed = RPM.of(-2000);

      public static final Angle kDeployedPosition  = Radians.of(Math.PI / 2);
      public static final Angle kRetractedPosition = Radians.zero();
    }
  }

  public static final class IndexerConstants {
    public static final int kMotorCanId = 12;

    public static final class Control {
      public static final Angle kAllowableError = Radians.of(Math.PI / 8);

      public static final Angle kStepSize = Radians.of(Math.PI / 2);

      public static final Time kStepWaitTime = Seconds.of(0.25);
    }
  }

  public static final class ShooterConstants {
     public static final int kMotorCanId = 9;

     public static final AngularVelocity kFreeSpeed = RPM.of(6200);

     public static final class Control {
      public static final AngularVelocity kAllowableError = RPM.of(100);
     }
    /* Good value of 300 to make ball not super fast */
     public static final AngularVelocity kTargetSpeed = RPM.of(360);
  }

  public static final class ColorSensorConstants {
    public static final int kSensorId = 0;

    public static final int kHistoryBufferSize = 256;
  }

  public static final class TrackerConstants {
    public static final int kDerivativeWindow = 15; // may need adjustments idk
    public static final int calculateEndShift(int x) {return (x * 2) - 1;}

    public static final Time kDebounceTimeSeconds = Milliseconds.of(300); // will need to be adjusted. 0.5f low frequency 0.2f high frequency
    public static final float kAllowableError = 0.1f;

    public static final Time kSettleTime = Seconds.of(2); // Block detections on boot for this amount of time to allow buffers to populate before making decisions
  }

  public static final class GyroConstants {
    public static final Angle kFlatThreshold = Radians.of(Math.PI / 8);

    public static final Angle kTargetAngleAllowableError = Radians.of(Math.PI / 8);
// TODO Need to doublecheck this is used all over, references for ID 10 and 21 are 
    public static final int kPigeonPort = 10;
  }

  public static final class ModuleConstants {
    public static final Distance kWheelDiameter = Meters.of(0.0762);
    public static final Distance kWheelCircumference = kWheelDiameter.times(Math.PI);

    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorReduction = (45.0 * 16.0 * 50.0) / (kDrivingMotorPinionTeeth * 15.0 * 28.0); // L3, see https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public static final AngularVelocity kDriveWheelFreeSpeed = RPM.of((NeoMotorContants.kNeoFreeSpeed.times(kWheelCircumference).magnitude()) / kDrivingMotorReduction);

    public static final double kDriveFactor = ModuleConstants.kWheelDiameter.abs(Meters) * Math.PI / ModuleConstants.kDrivingMotorReduction;
    public static final double kTurnFactor  = 2 * Math.PI;

    public static final double kDriveMOI = 0.025;
    public static final double kTurnMOI  = 0.004;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final int kSimulationJoystickPort = 1;

    public static final double kDriveDeadband = 0.03;
  }

  public static final class AutoConstants {
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(3); // 3;
    public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(3);
    public static final AngularAcceleration kMaxAngularAcceleration = RadiansPerSecondPerSecond.of(Math.PI);
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(Math.PI);

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularAcceleration.abs(RadiansPerSecondPerSecond), kMaxAngularSpeed.abs(RadiansPerSecond));
  }

  public static final class NeoMotorContants {
    public static final AngularVelocity kNeoFreeSpeed = RPM.of(5676);
    public static final AngularVelocity kVortexFreeSpeedRpm = RPM.of(6784);
  }

  public static final class PhotonVisionConstants {

    // NE FIRST uses the AndyMark field. See Table 5-1: District Field Types at https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
    public static final AprilTagFieldLayout kApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Refer to https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // and https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
    public static final Transform3d kRobotToCam0 = new Transform3d(
      new Translation3d(
        0.5, // x distance offset from the center of the robot (forward) in meters
        0.0, // y distance offset from the center of the robot (left) in meters
        0.5 // height of the camera from the floor in meters
      ),
      new Rotation3d(
        0, // roll (usually 0 unless you have a special case) in radians
        0, // tilt angle of the camera (where negative sign is looking up) in radians
        0  // yaw in radians (0 if the camera is facing forward, Pi if the camera is facing backward)
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

  public static final class LedConfigs {
    public static final LedStripConfig strip1 = new LedStripConfig(0, 30, 60);
  }

  // public static interface BallPhysicsSimConstants {
  //   Angle kLaunchAngle = Degrees.of(73);
  //   Distance kWheelRadius = Inches.of(2);
  //   Distance kLaunchHeight = Inches.of(16);
  //   double kRollCoefficient = 0.5;
  //   double kDragCoefficient = 0.5;
  //   double kLiftCoefficient = 0;
  //   int    kMaxIterations   = 500;

  //   Distance kBallRadius = Inches.of(6);
  //   LinearVelocity kGravity = MetersPerSecond.of(9.81);
  //   // Massnet kBallMass = 
    
  // }
}
