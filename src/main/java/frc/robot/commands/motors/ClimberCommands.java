package frc.robot.commands.motors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.generics.GenericMotorControl;
import frc.robot.subsystems.motors.Climber;

public interface ClimberCommands {
  interface Await {
    interface Extend {
      public class Actively extends GenericMotorControl.Position.Await.Actively {
        public Actively(Climber climber) {
          super(climber, ClimberConstants.Control.ExtendedPosition);
        }
      }

      static Command Actively(Climber climber) {
        return new Actively(climber);
      }

      public class Passively extends GenericMotorControl.Position.Await.Passively {
        public Passively(Climber climber) {
          super(climber, ClimberConstants.Control.ExtendedPosition);
        }
      }

      static Command Passively(Climber climber) {
        return new Passively(climber);
      }
    }

    interface Retract {
      public class Actively extends GenericMotorControl.Position.Await.Actively {
        public Actively(Climber climber) {
          super(climber, ClimberConstants.Control.RetractedPosition);
        }
      }

      static Command Actively(Climber climber) {
        return new Actively(climber);
      }

      public class Passively extends GenericMotorControl.Position.Await.Passively {
        public Passively(Climber climber) {
          super(climber, ClimberConstants.Control.RetractedPosition);
        }
      }

      static Command Passively(Climber climber) {
        return new Passively(climber);
      }
    }

  }
}
