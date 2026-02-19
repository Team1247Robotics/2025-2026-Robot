package frc.robot.subsystems;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.generics.GenericNeo;

public class Shooter extends GenericNeo {
  public Shooter() {
    super(ShooterConstants.kMotorCanId, Configs.ShooterMotor.config);
  }
}
