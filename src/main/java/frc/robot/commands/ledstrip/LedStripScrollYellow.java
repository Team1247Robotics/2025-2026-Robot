package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.bases.LedStripBaseCommand;
import frc.robot.subsystems.LedStrip;

public class LedStripScrollYellow extends LedStripBaseCommand {

  public LedStripScrollYellow(LedStrip strip) {super(strip);}

  @Override
  public void execute() {
    m_strip.applyGradient(GradientType.kContinuous, Color.kBlack, Color.kYellow, Color.kBlack);
  }
}
