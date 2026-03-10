package frc.robot.commands.motors;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.generics.GenericMotorControl;
import frc.robot.subsystems.motors.UpperShooterFeeder;

public interface FeederCommands {
  public class Stop extends GenericMotorControl.Stop {
    public Stop(UpperShooterFeeder feeder) {
      super(feeder);
    }
  }
  static Command Stop(UpperShooterFeeder feeder) {
    return new Stop(feeder);
  }

  interface Await {
    class Actively extends GenericMotorControl.Velocity.Await.Actively {
      public Actively(UpperShooterFeeder feeder) {
        super(feeder, FeederConstants.Control.kTargetSpeed.in(RPM));
      }
    }

    static Command Actively(UpperShooterFeeder feeder) {
      return new Actively(feeder);
    }

    class Passively extends GenericMotorControl.Velocity.Await.Passively {
      public Passively(UpperShooterFeeder feeder) {
        super(feeder, FeederConstants.Control.kTargetSpeed.in(RPM));
      }
    }

    static Command Passively(UpperShooterFeeder feeder) {
      return new Passively(feeder);
    }
  }
  class Run extends GenericMotorControl.Velocity.Set {
    public Run(UpperShooterFeeder feeder) {
      super(feeder, FeederConstants.Control.kTargetSpeed.in(RPM));
    }
  }

  static Command Run(UpperShooterFeeder feeder) {
    return new Run(feeder);
  }
}
