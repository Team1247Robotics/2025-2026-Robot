package frc.robot;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.NeoMotorContants;
import frc.robot.Constants.ShooterConstants;

public final class Configs {
  public static final class SDSSwerveModule {
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeed.in(RPM);

      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      drivingConfig.encoder
        .positionConversionFactor(ModuleConstants.kDriveFactor)
        .velocityConversionFactor(ModuleConstants.kDriveFactor / 60.0);

      drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.01, 0.0, 0)
        .outputRange(-1, 1)
        .feedForward.kV(drivingVelocityFeedForward/* * 0.1025*/);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

      
      turningConfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(ModuleConstants.kTurnFactor)
        .velocityConversionFactor(ModuleConstants.kTurnFactor / 60.0)
        .averageDepth(128)
        .startPulseUs(0)
        .endPulseUs(0);
      
      turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.9, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, ModuleConstants.kTurnFactor);
    }
  }

  public static final class IndexerMotor {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeed.in(RPM);

      config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50);

      config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot0) // Position
        .pid(0, 0.0, 0, ClosedLoopSlot.kSlot1) // Velocity
        .outputRange(-1, 1)
        .feedForward.kV(feedForward, ClosedLoopSlot.kSlot1);
    }
  }

  public static final class BeltFeederConfig {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeed.in(RPM);

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


  public static final class UpperShooterFeederConfig {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeed.in(RPM);

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

public static final class LowerShooterFeederConfig {
    public static final SparkFlexConfig config = new SparkFlexConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeed.in(RPM);

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
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeed.in(RPM);

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

  public static final class IntakeMotor {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / (IntakeConstants.kMotorFreeSpeed.in(RPM));

      config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .inverted(false); // true

      config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot0) // Position
        .pid(0.0, 0.0, 0, ClosedLoopSlot.kSlot1) // Velocity
        .outputRange(-1, 1)
        .feedForward.kV(feedForward * 0.085, ClosedLoopSlot.kSlot1);
    }
  }
  
  public static final class IntakeDeploymentMotor {
    public static final SparkMaxConfig config = new SparkMaxConfig();

    static {
      double nominalVoltage = 12;
      double feedForward = nominalVoltage / NeoMotorContants.kNeoFreeSpeed.in(RPM);

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
      double feedForward = nominalVoltage / ShooterConstants.kFreeSpeed.in(RPM);

      config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .inverted(true);

      config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.12, 0.0, 0, ClosedLoopSlot.kSlot0) // Position
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1) // Velocity
        .outputRange(-1, 1)
        .feedForward.kV(feedForward, ClosedLoopSlot.kSlot1);
    }
  }
}
