package frc.robot.subsystems.motors;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.generics.GenericVortex;

public class ShooterFollower extends GenericVortex {
  public ShooterFollower() {
    super(ShooterConstants.kFollowerMotorCanId, Configs.ShooterFollowerMotor.config);
  }

  @Override
  public void periodic() {
    sendStatsToDash("ShooterFollower");
  }
}
