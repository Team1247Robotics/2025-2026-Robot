package frc.robot.commands.bases;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStrip;

public class LedStripBaseCommand extends Command {
  protected final LedStrip m_strip;
  private boolean m_ignoreDisable = true;

  public LedStripBaseCommand(LedStrip strip) {
    m_strip = strip;
    addRequirements(strip);
    this.ignoringDisable(m_ignoreDisable);
  }

  public void setIgnoreDisable(boolean value) {
    m_ignoreDisable = value;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_ignoreDisable;
  }
}
