package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

public class Shooter extends GenericSparkMaxMotor {
  public Shooter() {
    super(ShooterConstants.kMotorCanId, MotorType.kBrushless, Configs.ShooterMotor.config);
  }
}
