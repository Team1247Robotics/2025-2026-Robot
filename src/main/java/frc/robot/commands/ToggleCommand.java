package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleCommand extends Command {
  private boolean m_state = false;
  public ToggleCommand() {}

  public boolean getState() {
    return m_state;
  }

  public Trigger getTrigger() {
    return new Trigger(this::getState);
  }

  public Command enable() {
    return Commands.runOnce(() -> m_state = true, new Subsystem[] {});
  }

  public Command disable() {
    return Commands.runOnce(() -> m_state = false, new Subsystem[] {});
  }
}
