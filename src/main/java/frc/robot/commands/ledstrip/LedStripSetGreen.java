package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStrip;

public class LedStripSetGreen extends Command {
    private final LedStrip m_ledStrip;

    public LedStripSetGreen(LedStrip ledStrip) {
        m_ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void execute() {
        m_ledStrip.setSolidColor(Color.kGreen);
    }
}