package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.GenericConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

public interface GenericMotorControl {
  class Stop extends Command {
    protected final GenericSparkMaxMotor m_motor;
    public Stop(GenericSparkMaxMotor motor) {
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
        protected final GenericSparkMaxMotor m_motor;
        public Passively(GenericSparkMaxMotor motor, DoubleSupplier targetSupplier) {
          super(motor::getVelocity, targetSupplier, GenericConstants.MotorPositionControlAllowableError);
          m_motor = motor;
        }
  
        public Passively(GenericSparkMaxMotor motor, double target) {
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
        public Actively(GenericSparkMaxMotor motor, DoubleSupplier target) {
          super(motor, target);
          addRequirements(motor);
        }
  
        /**
         * Instatiate with a statically set value.
         * @param motor - The shooter object
         * @param target - A double representing the target position.
         */
        public Actively(GenericSparkMaxMotor motor, double target) {
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
      public Set(GenericSparkMaxMotor motor, DoubleSupplier target) {
        super(motor, target);
      }
      
      /**
       * Instatiate with a statically set value.
       * @param motor - The shooter object
       * @param target - A double representing the target position.
       */
      public Set(GenericSparkMaxMotor motor, double target) {
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
        protected final GenericSparkMaxMotor m_motor;
        public Passively(GenericSparkMaxMotor motor, DoubleSupplier targetSupplier) {
          super(motor::getVelocity, targetSupplier, GenericConstants.MotorVelocityControlAllowableError);
          m_motor = motor;
          addRequirements(new Subsystem[0]);
        }

        public Passively(GenericSparkMaxMotor motor, double target) {
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
        public Actively(GenericSparkMaxMotor motor, DoubleSupplier target) {
          super(motor, target);
          addRequirements(motor);
        }

        /**
         * Instatiate with a statically set value.
         * @param motor - The shooter object
         * @param target - A double representing the target velocity in RPM.
         */
        public Actively(GenericSparkMaxMotor motor, double target) {
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
      public Set(GenericSparkMaxMotor motor, DoubleSupplier target) {
        super(motor, target);
      }

      /**
       * Instatiate with a statically set value.
       * @param motor - The shooter object
       * @param target - A double representing the target velocity.
       */
      public Set(GenericSparkMaxMotor motor, double target) {
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
