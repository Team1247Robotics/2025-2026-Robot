package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStrip;

public class LedStripScrollYellow extends Command {
    private final LedStrip m_ledStrip;

    public LedStripScrollYellow(LedStrip ledStrip) {
        m_ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void execute() {
        m_ledStrip.showYellow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
