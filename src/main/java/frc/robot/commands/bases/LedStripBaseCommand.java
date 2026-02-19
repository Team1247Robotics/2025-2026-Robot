package frc.robot.commands.bases;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStrip;

public class LedStripBaseCommand extends Command {
  protected final LedStrip m_strip;

  public LedStripBaseCommand(LedStrip strip) {
    m_strip = strip;
    addRequirements(strip);
    this.ignoringDisable(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
