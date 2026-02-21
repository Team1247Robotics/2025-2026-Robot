package frc.robot.commands.motors;

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

      public class Passively extends GenericMotorControl.Position.Await.Passively {
        public Passively(Climber climber) {
          super(climber, ClimberConstants.Control.ExtendedPosition);
        }
      }
    }

    interface Retract {
      public class Actively extends GenericMotorControl.Position.Await.Actively {
        public Actively(Climber climber) {
          super(climber, ClimberConstants.Control.RetractedPosition);
        }
      }

      public class Passively extends GenericMotorControl.Position.Await.Passively {
        public Passively(Climber climber) {
          super(climber, ClimberConstants.Control.RetractedPosition);
        }
      }
    }

  }
}
