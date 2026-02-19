package frc.robot.commands.ledstrip;

import frc.robot.commands.bases.LedStripBaseCommand;
import frc.robot.subsystems.LedStrip;

public class LedStripScrollRainbow extends LedStripBaseCommand {
  public LedStripScrollRainbow(LedStrip ledStrip) {
    super(ledStrip);
  }

  @Override
  public void execute() {
    m_strip.applyRainbow(255, 255);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
