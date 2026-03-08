package frc.robot.commands.generics;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.GenericConstants;
import frc.robot.subsystems.generics.GenericSparkFlexMotor;

public interface GenericFlexMotorControl {
  class Stop extends Command {
    protected final GenericSparkFlexMotor m_motor;
    public Stop(GenericSparkFlexMotor motor) {
      m_motor = motor;
      addRequirements(motor);
    }

    @Override
    public void execute() {
      m_motor.stop();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
  interface Position {
    interface Await {
      /**
       * A command that does nothing and will only complete when the given motor reaches the given position.
       * @implNote Motor can be accessed under {@link #m_motor}, target can be accessed under {@link #m_target}.
       */
      public class Passively extends GenericAwaitBaseTargetWithinError {
        protected final GenericSparkFlexMotor m_motor;
        public Passively(GenericSparkFlexMotor motor, DoubleSupplier targetSupplier) {
          super(motor::getVelocity, targetSupplier, GenericConstants.kMotorPositionControlAllowableError.in(Radians));
          m_motor = motor;
        }
  
        public Passively(GenericSparkFlexMotor motor, double target) {
          this(motor, () -> target);
        }
      }
  
      /**
       * Command that spins motor to set velocity until reaching position. Will finish when the target has been reached.
       */
      public class Actively extends Passively {
        
        /**
         * Instantiate with a dynamically updating value.
         * @param motor - The shooter object
         * @param target - A double supplier that will return the latest target position.
         */
        public Actively(GenericSparkFlexMotor motor, DoubleSupplier target) {
          super(motor, target);
          addRequirements(motor);
        }
  
        /**
         * Instatiate with a statically set value.
         * @param motor - The shooter object
         * @param target - A double representing the target position.
         */
        public Actively(GenericSparkFlexMotor motor, double target) {
          super(motor, target);
          addRequirements(motor);
        }
  
        @Override
        public void execute() {
          m_motor.setPosition(m_target.getAsDouble());
        }
      }
    }
  
    public static class Set extends Await.Actively {
      /**
       * Instantiate with a dynamically updating value.
       * @param motor - The shooter object
       * @param target - A double supplier that will return the latest target position.
       */
      public Set(GenericSparkFlexMotor motor, DoubleSupplier target) {
        super(motor, target);
      }
      
      /**
       * Instatiate with a statically set value.
       * @param motor - The shooter object
       * @param target - A double representing the target position.
       */
      public Set(GenericSparkFlexMotor motor, double target) {
        super(motor, () -> target);
      }
  
      @Override
      public boolean isFinished() {
        return false;
      }
    }
  }

  interface Velocity {
    interface Await {
      /**
       * A command that does nothing and will only complete when the given motor reaches the given speed.
       * @implNote Motor can be accessed under {@link #m_motor}, target can be accessed under {@link #m_target}.
       */
      public class Passively extends GenericAwaitBaseTargetWithinError {
        protected final GenericSparkFlexMotor m_motor;
        public Passively(GenericSparkFlexMotor motor, DoubleSupplier targetSupplier) {
          super(motor::getVelocity, targetSupplier, GenericConstants.kMotorVelocityControlAllowableError.in(RPM));
          m_motor = motor;
          addRequirements(new Subsystem[0]);
        }

        public Passively(GenericSparkFlexMotor motor, double target) {
          this(motor, () -> target);
        }
      }

      /**
       * Command that spins motor to set velocity until reaching velocity. Will finish when the target has been reached.
       */
      public class Actively extends Passively {
        
        /**
         * Instantiate with a dynamically updating value.
         * @param motor - The shooter object
         * @param target - A double supplier that will return the latest target velocity in RPM every tick.
         */
        public Actively(GenericSparkFlexMotor motor, DoubleSupplier target) {
          super(motor, target);
          addRequirements(motor);
        }

        /**
         * Instatiate with a statically set value.
         * @param motor - The shooter object
         * @param target - A double representing the target velocity in RPM.
         */
        public Actively(GenericSparkFlexMotor motor, double target) {
          super(motor, target);
          addRequirements(motor);
        }

        @Override
        public void execute() {
          m_motor.setVelocity(m_target.getAsDouble());
        }
      }
    }

    /**
     * Command that spins motor at set velocity.
     */
    public class Set extends Await.Actively {
      /**
       * Instantiate with a dynamically updating value.
       * @param motor - The motor object
       * @param target - A double supplier that will return the latest target velocity.
       */
      public Set(GenericSparkFlexMotor motor, DoubleSupplier target) {
        super(motor, target);
      }

      /**
       * Instatiate with a statically set value.
       * @param motor - The shooter object
       * @param target - A double representing the target velocity.
       */
      public Set(GenericSparkFlexMotor motor, double target) {
        super(motor, target);
      }

      @Override
      public void execute() {
        m_motor.setVelocity(m_target.getAsDouble());
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    }
  }
}
