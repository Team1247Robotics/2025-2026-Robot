package frc.robot.commands.motors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.generics.GenericMotorControl;
import frc.robot.subsystems.motors.Shooter;

public interface ShooterCommands {
  interface ShooterDependant {
    public class Sequence extends SequentialCommandGroup {
      public Sequence(Shooter shooter, DoubleSupplier speed, Command... commands) {
        super(
          new ShooterCommands.Run.Await.Actively(shooter, speed),
          Commands.parallel(
            new ShooterCommands.Run.Indefinitely(shooter, speed),
            Commands.sequence(commands)
          )
        );
      }

      public Sequence(Shooter shooter, Double speed, Command... commands) {
        this(shooter, () -> speed, commands);
      }

      public Sequence(Shooter shooter, Command... commands) {
        this(shooter, ShooterConstants.targetSpeed, commands);
      }
    }

    public static Command Sequence(Shooter shooter, DoubleSupplier speed, Command... commands) {
      return new Sequence(shooter, speed, commands);
    }

    public static Command Sequence(Shooter shooter, double speed, Command... commands) {
      return new Sequence(shooter, speed, commands);
    }

    public static Command Sequence(Shooter shooter, Command... commands) {
      return new Sequence(shooter, commands);
    }

    public class Parallel extends SequentialCommandGroup {
      public Parallel(Shooter shooter, DoubleSupplier speed, Command... commands) {
        super(
          new ShooterCommands.Run.Await.Actively(shooter, speed),
          Commands.parallel(
            new ShooterCommands.Run.Indefinitely(shooter, speed),
            Commands.parallel(commands)
          )
        );
      }

      public Parallel(Shooter shooter, Double speed, Command... commands) {
        this(shooter, () -> speed, commands);
      }

      public Parallel(Shooter shooter, Command... commands) {
        this(shooter, ShooterConstants.targetSpeed, commands);
      }
    }

    public static Command Parallel(Shooter shooter, DoubleSupplier speed, Command... commands) {
      return new Parallel(shooter, speed, commands);
    }

    public static Command Parallel(Shooter shooter, double speed, Command... commands) {
      return new Parallel(shooter, speed, commands);
    }

    public static Command Parallel(Shooter shooter, Command... commands) {
      return new Parallel(shooter, commands);
    }
  }

  interface Run {
    interface Await {
      /**
       * Command that spins shooter to set velocity until reaching velocity. Will finish when the target has been reached.
       */
      public class Actively extends GenericMotorControl.Velocity.Await.Actively {
        
        /**
         * Instantiate with a dynamically updating value.
         * @param shooter - The shooter object
         * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
         */
        public Actively(Shooter shooter, DoubleSupplier velocity) {
          super(shooter, velocity);
        }

        /**
         * Instatiate with a statically set value.
         * @param shooter - The shooter object
         * @param velocity - A double representing the target velocity in RPM.
         */
        public Actively(Shooter shooter, double velocity) {
          super(shooter, velocity);
        }

        public Actively(Shooter shooter) {
          this(shooter, ShooterConstants.targetSpeed);
        }
      }

      /**
       * Instantiate with a dynamically updating value.
       * @param shooter - The shooter object
       * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
       */
      public static Command Actively(Shooter shooter, DoubleSupplier velocity) {
        return new Actively(shooter, velocity);
      }

      /**
       * Instatiate with a statically set value.
       * @param shooter - The shooter object
       * @param velocity - A double representing the target velocity in RPM.
       */
      public static Command Actively(Shooter shooter, double velocity) {
        return new Actively(shooter, velocity);
      }

      /**
       * Instatiate with a statically set value.
       * @param shooter - The shooter object
       */
      public static Command Actively(Shooter shooter) {
        return new Actively(shooter);
      }

      /**
       * Command that waits for shooter to reach velocity. Will finish when the target has been reached.
       */
      public class Passively extends GenericMotorControl.Velocity.Await.Passively {
        
        /**
         * Instantiate with a dynamically updating value.
         * @param shooter - The shooter object
         * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
         */
        public Passively(Shooter shooter, DoubleSupplier velocity) {
          super(shooter, velocity);
        }

        /**
         * Instatiate with a statically set value.
         * @param shooter - The shooter object
         * @param velocity - A double representing the target velocity in RPM.
         */
        public Passively(Shooter shooter, double velocity) {
          super(shooter, velocity);
        }
      }

      /**
       * Instantiate with a dynamically updating value.
       * @param shooter - The shooter object
       * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
       */
      public static Command Passively(Shooter shooter, DoubleSupplier velocity) {
        return new Actively(shooter, velocity);
      }

      /**
       * Instatiate with a statically set value.
       * @param shooter - The shooter object
       * @param velocity - A double representing the target velocity in RPM.
       */
      public static Command Passively(Shooter shooter, double velocity) {
        return new Actively(shooter, velocity);
      }
    }

    /**
     * Indefinitely holds the speed of the shooter until forcefully interrupted.
     */
    class Indefinitely extends GenericMotorControl.Velocity.Set {
      /**
       * Instantiate with a dynamically updating value.
       * @param shooter - The shooter object
       * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
       */
      public Indefinitely(Shooter shooter, DoubleSupplier velocity) {
        super(shooter, velocity);
      }

      /**
       * Instatiate with a statically set value.
       * @param shooter - The shooter object
       * @param velocity - A double representing the target velocity in RPM.
       */
      public Indefinitely(Shooter shooter, double velocity) {
        super(shooter, velocity);
      }

      /**
       * Instatiate with a statically set value.
       * @param shooter - The shooter object
       */
      public Indefinitely(Shooter shooter) {
        this(shooter, ShooterConstants.targetSpeed);
      }
    }

    /**
     * Instantiate with a dynamically updating value.
     * @param shooter - The shooter object
     * @param velocity - A double supplier that will return the latest target velocity in RPM every tick.
     */
    static Command Indefinitely(Shooter shooter, DoubleSupplier velocity) {
      return new Indefinitely(shooter, velocity);
    }

    /**
     * Instatiate with a statically set value.
     * @param shooter - The shooter object
     * @param velocity - A double representing the target velocity in RPM.
     */
    static Command Indefinitely(Shooter shooter, double velocity) {
      return new Indefinitely(shooter, velocity);
    }

    /**
     * Instatiate with a statically set value.
     * @param shooter - The shooter object
     */
    static Command Indefinitely(Shooter shooter) {
      return new Indefinitely(shooter, ShooterConstants.targetSpeed);
    }
  }
}