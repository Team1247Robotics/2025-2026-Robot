package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShooterDependantSequence extends SequentialCommandGroup {
  public ShooterDependantSequence(Shooter shooter, DoubleSupplier speed, Command... commands) {
    super(
      new ArmShooterBlocking(shooter, speed),
      Commands.parallel(
        new ArmShooterAsync(shooter, speed),
        Commands.sequence(commands)
      )
    );
  }
}
