package frc.robot;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.NeoMotorContants;

public final class Configs {
  public static final class SDSSwerveModule {
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

    static {
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;

      double turningFactor = 2 * Math.PI;
      double nominalVoltage = 12;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      drivingConfig.encoder
        .positionConversionFactor(drivingFactor)
        .velocityConversionFactor(drivingFactor / 60.0);

      drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0)
        .outputRange(-1, 1)
        .feedForward.kV(drivingVelocityFeedForward);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

      
      turningConfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(turningFactor)
        .velocityConversionFactor(turningFactor / 60.0)
        .averageDepth(128)
        .startPulseUs(0)
        .endPulseUs(0);
      
      turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.9, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class IndexerMotor {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeedRpm;

      config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot0) // Position
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot1) // Velocity
        .outputRange(-1, 1)
        .feedForward.kV(feedForward, ClosedLoopSlot.kSlot1);
    }
  }

  public static final class FeederMotor {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeedRpm;

      config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot0) // Position
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot1) // Velocity
        .outputRange(-1, 1)
        .feedForward.kV(feedForward, ClosedLoopSlot.kSlot1);
    }
  }

  public static final class ClimberMotor {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeedRpm;

      config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot0) // Position
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot1) // Velocity
        .outputRange(-1, 1)
        .feedForward.kV(feedForward, ClosedLoopSlot.kSlot1);
    }
  }

  public static final class ShooterMotor {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeedRpm;

      config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50);

      config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot0) // Position
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot1) // Velocity
        .outputRange(-1, 1)
        .feedForward.kV(feedForward, ClosedLoopSlot.kSlot1);
    }
  }
}
