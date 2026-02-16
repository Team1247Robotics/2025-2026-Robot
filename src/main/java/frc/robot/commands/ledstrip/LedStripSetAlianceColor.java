package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LedStrip;
import frc.robot.utils.GetAlliance;
import frc.robot.utils.HubActiveState;

public class LedStripSetAlianceColor extends LedStripBaseCommand {
  private enum LightStates {
    HubActive,
    BlueAlliance,
    RedAlliance,
    None
  }

  private LightStates m_lastState = LightStates.None;

  public LedStripSetAlianceColor(LedStrip ledStrip) {
    super(ledStrip);
  }

  private void updateState(LightStates newState) {
    switch (newState) {
      case HubActive:
        m_strip.setSolidColor(Color.kViolet);
        break;
      case BlueAlliance:
        m_strip.setSolidColor(Color.kBlue);
        break;
      case RedAlliance:
        m_strip.setSolidColor(Color.kRed);
        break;
      default:
        // idk
        break;
    }
    m_lastState = newState;
  }

  private LightStates composeState() {
    if (HubActiveState.getInstance().isOurHubActive()) return LightStates.HubActive;
    if (GetAlliance.isBlueAlliance()) return LightStates.BlueAlliance;
    if (GetAlliance.isRedAlliance()) return LightStates.RedAlliance;
    return LightStates.None;
  }

  @Override
  public void execute() {
    var latestState = composeState();
    if (latestState.equals(m_lastState)) return;
    updateState(latestState);
  }
}
