package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.motors.Shooter;

public class ShooterDependantParallel extends SequentialCommandGroup {
  public ShooterDependantParallel(Shooter shooter, DoubleSupplier speed, Command... commands) {
    super(
      new SpinUpShooter(shooter, speed),
      Commands.parallel(
        new HoldShooterSpeed(shooter, speed),
        Commands.parallel(commands)
      )
    );
  }
}
