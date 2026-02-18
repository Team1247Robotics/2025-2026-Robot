package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStrip;

public class LedStripBaseCommand extends Command {
  protected final LedStrip m_strip;

  public LedStripBaseCommand(LedStrip strip) {
    m_strip = strip;
    addRequirements(strip);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
