package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedStrip;
import frc.robot.utils.GetAlliance;
import frc.robot.utils.HubActiveState;

public class LedStripSetAlianceColor extends Command{
    private final LedStrip m_ledStrip;

    public LedStripSetAlianceColor(LedStrip ledStrip) {
        m_ledStrip = ledStrip;
        addRequirements(ledStrip);
    }

    @Override
    public void execute() {
        if (HubActiveState.getInstance().isOurHubActive()){
            m_ledStrip.setSolidColor(Color.kViolet);
        }else if (GetAlliance.isBlueAlliance()) {
            m_ledStrip.setSolidColor(Color.kBlue);
        }else if (GetAlliance.isRedAlliance()) {
            m_ledStrip.setSolidColor(Color.kRed);
        
        }
    }
}
