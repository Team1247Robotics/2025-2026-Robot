package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStrip;

public class LedStripScrollRainbow extends Command {
    private final LedStrip m_ledStrip;

    public LedStripScrollRainbow(LedStrip ledStrip) {
        m_ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void execute() {
        m_ledStrip.showRainbow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
