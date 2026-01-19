package frc.robot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class SDSSwerveModule {
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    
        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
    
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFeeSpeedRps;
    
            drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
    
            drivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);
    
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.14, 0.002, 0)
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
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }
    }
}
