package frc.robot.commands.motors;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.generics.GenericMotorControl;
import frc.robot.subsystems.motors.Intake;
import frc.robot.subsystems.motors.IntakeDeployment;

public interface IntakeCommands {
  interface Driver {
    public interface Run {
      public interface Await {
        public static class Actively extends GenericMotorControl.Velocity.Await.Actively {
          public Actively(Intake intake) {
            super(intake, IntakeConstants.Control.IntakeSpeed);
          }
        }
      
        public static class Passively extends GenericMotorControl.Velocity.Await.Passively {
          public Passively(Intake intake) {
            super(intake, IntakeConstants.Control.IntakeSpeed);
          }
        }
      }

      public class Indefinitely extends GenericMotorControl.Velocity.Set {
        public Indefinitely(Intake intake) {
          super(intake, IntakeConstants.Control.IntakeSpeed);
        }
      }
    }
  }

  interface Deployment {
    interface Await {
      public static interface Deploy {
        
        public static class Actively extends GenericMotorControl.Position.Await.Actively {
          public Actively(IntakeDeployment intakeDeployment) {
            super(intakeDeployment, IntakeConstants.Control.DeployedPosition);
          }
        }
    
        public static class Passively extends GenericMotorControl.Position.Await.Passively {
          public Passively(IntakeDeployment intakeDeployment) {
            super(intakeDeployment, IntakeConstants.Control.DeployedPosition);
          }
        }
      }
    
      public static interface Retract {
        public static class Actively extends GenericMotorControl.Position.Await.Actively {
          public Actively(IntakeDeployment intakeDeployment) {
            super(intakeDeployment, IntakeConstants.Control.RetractedPosition);
          }
        }
    
        public static class Passively extends GenericMotorControl.Position.Await.Passively {
          public Passively(IntakeDeployment intakeDeployment) {
            super(intakeDeployment, IntakeConstants.Control.RetractedPosition);
          }
        }
      }
    }
  }
}
