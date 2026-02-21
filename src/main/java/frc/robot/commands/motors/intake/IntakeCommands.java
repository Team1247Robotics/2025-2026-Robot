package frc.robot.commands.motors.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.generics.GenericActiveAwaitMotorSpeed;
import frc.robot.commands.generics.GenericPassiveAwaitMotorSpeed;
import frc.robot.commands.generics.GenericSetMotorSpeed;
import frc.robot.subsystems.motors.Intake;

public interface IntakeCommands {
  public interface Run {
    public interface Await {
      public static class Actively extends GenericActiveAwaitMotorSpeed {
        public Actively(Intake intake) {
          super(intake, IntakeConstants.Control.IntakeSpeed);
        }
      }
    
      public static class Passively extends GenericPassiveAwaitMotorSpeed {
        public Passively(Intake intake) {
          super(intake, IntakeConstants.Control.IntakeSpeed);
        }
      }
    }

    public class Indefinitely extends GenericSetMotorSpeed {
      public Indefinitely(Intake intake) {
        super(intake, IntakeConstants.Control.IntakeSpeed);
      }
    }
  }
}
