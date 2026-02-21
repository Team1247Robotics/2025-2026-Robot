package frc.robot.commands.motors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.generics.GenericMotorControl;
import frc.robot.subsystems.motors.Feeder;

public interface FeederCommands {
  interface Await {
    class Actively extends GenericMotorControl.Velocity.Await.Actively {
      public Actively(Feeder feeder) {
        super(feeder, FeederConstants.Control.TargetSpeed);
      }
    }

    class Passively extends GenericMotorControl.Velocity.Await.Passively {
      public Passively(Feeder feeder) {
        super(feeder, FeederConstants.Control.TargetSpeed);
      }
    }
  }
  class Run extends GenericMotorControl.Velocity.Set {
    public Run(Feeder feeder) {
      super(feeder, FeederConstants.Control.TargetSpeed);
    }
  }

  static Command Run(Feeder feeder) {
    return new Run(feeder);
  }
}
