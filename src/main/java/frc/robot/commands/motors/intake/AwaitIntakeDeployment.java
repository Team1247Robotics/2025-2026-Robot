package frc.robot.commands.motors.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.generics.GenericActiveAwaitMotorPosition;
import frc.robot.commands.generics.GenericPassiveAwaitMotorPosition;
import frc.robot.subsystems.motors.IntakeDeployment;

public interface AwaitIntakeDeployment {
  public static interface Deploy {
    
    public static class Actively extends GenericActiveAwaitMotorPosition {
      public Actively(IntakeDeployment intakeDeployment) {
        super(intakeDeployment, IntakeConstants.Control.DeployedPosition);
      }
    }

    public static class Passively extends GenericPassiveAwaitMotorPosition {
      public Passively(IntakeDeployment intakeDeployment) {
        super(intakeDeployment, IntakeConstants.Control.DeployedPosition);
      }
    }
  }

  public static interface Retract {
    public static class Actively extends GenericActiveAwaitMotorPosition {
      public Actively(IntakeDeployment intakeDeployment) {
        super(intakeDeployment, IntakeConstants.Control.RetractedPosition);
      }
    }

    public static class Passively extends GenericPassiveAwaitMotorPosition {
      public Passively(IntakeDeployment intakeDeployment) {
        super(intakeDeployment, IntakeConstants.Control.RetractedPosition);
      }
    }
  }
  
}
