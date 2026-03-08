package frc.robot.subsystems.motors;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.generics.GenericVortex;

public class Shooter extends GenericVortex {
  public Shooter() {
    super(ShooterConstants.kMotorCanId, Configs.ShooterMotor.config);
  }

  @Override
  public void periodic() {
    sendStatsToDash("Shooter");
  }
}
