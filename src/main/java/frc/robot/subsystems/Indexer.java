package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs.IndexerMotor;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

public class Indexer extends GenericSparkMaxMotor {
  public Indexer() {
    super(IndexerConstants.kMotorCanId, MotorType.kBrushless, IndexerMotor.config);
  }
}
