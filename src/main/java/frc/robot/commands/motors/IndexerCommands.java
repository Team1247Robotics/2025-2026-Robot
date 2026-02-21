package frc.robot.commands.motors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.bases.DoXNTimes;
import frc.robot.commands.generics.GenericMotorControl;
import frc.robot.subsystems.motors.Indexer;

public interface IndexerCommands {
  interface Position {
    interface Await {
      class Actively extends GenericMotorControl.Position.Await.Actively {
        public Actively(Indexer indexer, DoubleSupplier targetSupplier) {
          super(indexer, targetSupplier);
        }
        
        public Actively(Indexer indexer, double targetPosition) {
          super(indexer, () -> targetPosition);
        }
      }

      class Passively extends GenericMotorControl.Position.Await.Passively {
        public Passively(Indexer indexer, DoubleSupplier targetSupplier) {
          super(indexer, targetSupplier);
        }
        
        public Passively(Indexer indexer, double targetPosition) {
          super(indexer, () -> targetPosition);
        }
      }
    }

    class Set extends GenericMotorControl.Position.Set {
      public Set(Indexer indexer, DoubleSupplier targetSupplier) {
        super(indexer, targetSupplier);
      }
      
      public Set(Indexer indexer, double targetPosition) {
        super(indexer, () -> targetPosition);
      }
    }
  }

  interface Abstracts {
    /**
     * Steps the indexer by the indexer step size constant.
     */
    public static class Step extends IndexerCommands.Position.Await.Actively {
      public Step(Indexer indexer) {
        super(indexer, () -> indexer.getPosition() + IndexerConstants.Control.indexerStdStepSize);
        setUseStaticTarget(true);
      }
    }

    static Command Step(Indexer indexer) {
      return new Step(indexer);
    }

    public static class StepAndPause extends SequentialCommandGroup {
      public StepAndPause(Indexer indexer) {
        addCommands(
          new IndexerCommands.Abstracts.Step(indexer),
          Commands.waitSeconds(IndexerConstants.Control.stepWaitTime)
        );
      }
    }

    static Command StepAndPause(Indexer indexer) {
      return new StepAndPause(indexer);
    }

    public class StepNTimes extends DoXNTimes {
      public StepNTimes(Indexer indexer, int n) {
        super(
          n,
          new IndexerCommands.Abstracts.StepAndPause(indexer)
        );
      }
      
    }

    static Command StepNTimes(Indexer indexer, int n) {
      return new StepNTimes(indexer, n);
    }
  }
}
