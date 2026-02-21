package frc.robot.commands.bases;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DoXNTimes extends SequentialCommandGroup {
  private int loops = 0;

  protected DoXNTimes(int n, Command... commands) {
    addCommands(Commands.deadline(
      Commands.waitUntil(() -> loops >= n),
      Commands.repeatingSequence(
        Commands.sequence(commands),
        Commands.runOnce(() -> loops++, new Subsystem[0])
      )
    ));
  }
}
