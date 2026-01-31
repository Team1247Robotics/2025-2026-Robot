package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LonelyTalonFx extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(40);
    private final Orchestra m_player = new Orchestra();

    public LonelyTalonFx() {
        m_player.addInstrument(m_motor);
        
    }
    
    public void playBadApple() {
        if (!m_player.isPlaying()) {
            m_player.stop();
            m_player.loadMusic("bad_apple.chrp");
            m_player.play();
        }
    }
}
