package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.bases.LedStripBaseCommand;
import frc.robot.subsystems.LedStrip;

public class LedStripSetGreen extends LedStripBaseCommand {
  public LedStripSetGreen(LedStrip ledStrip) {
    super(ledStrip);
  }
  
  @Override
  public void initialize() {
    m_strip.applySolid(Color.kGreen);
  }
}