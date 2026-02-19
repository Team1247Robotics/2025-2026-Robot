package frc.robot.subsystems.generics;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

public class GenericNeo extends GenericSparkMaxMotor {
  public GenericNeo(int canId, SparkBaseConfig config) {
    super(canId, MotorType.kBrushless, config);
  }
}
