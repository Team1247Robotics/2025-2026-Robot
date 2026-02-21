package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.GenericConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

public interface GenericMotorPosition {
  interface Await {
    /**
     * A command that does nothing and will only complete when the given motor reaches the given position.
     * @implNote Motor can be accessed under {@link #m_motor}, target can be accessed under {@link #m_target}.
     */
    public class Passive extends GenericAwaitBaseTargetWithinError {
      protected final GenericSparkMaxMotor m_motor;
      public Passive(GenericSparkMaxMotor motor, DoubleSupplier targetSupplier) {
        super(motor::getVelocity, targetSupplier, GenericConstants.MotorPositionControlAllowableError);
        m_motor = motor;
      }

      public Passive(GenericSparkMaxMotor motor, double target) {
        this(motor, () -> target);
      }
    }
  }
}
