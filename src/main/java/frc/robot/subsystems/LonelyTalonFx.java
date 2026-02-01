package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LonelyTalonFx extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(40);
    private final Orchestra m_player = new Orchestra();

    private static final String[] sounds = {
        "usseewa.chrp",
        "bad_apple.chrp",
        "king.chrp",
        "flop_era.chrp",
        "iris_out.chrp"
    };

    private static final String[] names = {
        "Usseewa",
        "Bad Apple!",
        "King",
        "Flop Era",
        "Iris Out"
    };

    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // private final Selector

    public LonelyTalonFx() {
        m_player.addInstrument(m_motor);
        for (int i = 0; i < sounds.length; i++) {
            if (i == 0) {
                m_chooser.setDefaultOption(names[i], sounds[i]);
            } else {
                m_chooser.addOption(names[i], sounds[i]);
            }
        }
        SmartDashboard.putData("Lonely Talon", m_chooser);
        
    }

    public void stop() {
        m_player.stop();
    }
    
    public void playBadApple() {
        String selected = m_chooser.getSelected();
        m_player.loadMusic(selected);
        m_player.play();
    }
}
