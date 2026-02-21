package frc.robot.commands.motors.feeder;

import frc.robot.Constants.FeederConstants;
import frc.robot.commands.generics.GenericPassiveAwaitMotorSpeed;
import frc.robot.commands.generics.GenericSetMotorSpeed;
import frc.robot.subsystems.motors.Feeder;

public class RunFeeder extends GenericSetMotorSpeed {
  public static class PassivelyAwaitFeederReady extends GenericPassiveAwaitMotorSpeed {
    public PassivelyAwaitFeederReady(Feeder feeder) {
      super(feeder, FeederConstants.Control.TargetSpeed);
    }
  }
  
  public RunFeeder(Feeder feeder) {
    super(feeder, FeederConstants.Control.TargetSpeed);
  }
}
