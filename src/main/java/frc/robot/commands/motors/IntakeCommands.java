package frc.robot.commands.motors;

import edu.wpi.first.wpilibj2.command.Command;
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

        static Command Actively(Intake intake) {
          return new Actively(intake);
        }
      
        public static class Passively extends GenericMotorControl.Velocity.Await.Passively {
          public Passively(Intake intake) {
            super(intake, IntakeConstants.Control.IntakeSpeed);
          }
        }

        static Command Passively(Intake intake) {
          return new Passively(intake);
        }
      }

      public class Indefinitely extends GenericMotorControl.Velocity.Set {
        public Indefinitely(Intake intake) {
          super(intake, IntakeConstants.Control.IntakeSpeed);
        }
      }

      public static Command Indefinitely(Intake intake) {
        return new Indefinitely(intake);
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

        static Command Actively(IntakeDeployment intakeDeployment) {
          return new Actively(intakeDeployment);
        }
    
        public static class Passively extends GenericMotorControl.Position.Await.Passively {
          public Passively(IntakeDeployment intakeDeployment) {
            super(intakeDeployment, IntakeConstants.Control.DeployedPosition);
          }
        }

        static Command Passively(IntakeDeployment intakeDeployment) {
          return new Passively(intakeDeployment);
        }
      }
    
      public static interface Retract {
        public static class Actively extends GenericMotorControl.Position.Await.Actively {
          public Actively(IntakeDeployment intakeDeployment) {
            super(intakeDeployment, IntakeConstants.Control.RetractedPosition);
          }
        }

        static Command Actively(IntakeDeployment intakeDeployment) {
          return new Actively(intakeDeployment);
        }
    
        public static class Passively extends GenericMotorControl.Position.Await.Passively {
          public Passively(IntakeDeployment intakeDeployment) {
            super(intakeDeployment, IntakeConstants.Control.RetractedPosition);
          }
        }

        static Command Passively(IntakeDeployment intakeDeployment) {
          return new Passively(intakeDeployment);
        }
      }
    }
  }
}
