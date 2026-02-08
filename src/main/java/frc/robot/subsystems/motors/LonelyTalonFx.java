package frc.robot.subsystems.motors;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LonelyTalonFx extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(40);
    private final Orchestra m_player = new Orchestra();

    private record Sound(String file, String name) {}

    private static final Sound[] sounds = {
        new Sound("bad_apple.chrp", "Bad Apple!"),
        new Sound("usseewa.chrp", "Usseewa"),
        new Sound("king.chrp", "King"),
        new Sound("flop_era.chrp", "Flop Era"),
        new Sound("iris_out.chrp", "Iris Out")
    };

    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // private final Selector

    public LonelyTalonFx() {
        m_player.addInstrument(m_motor);
        for (int i = 0; i < sounds.length; i++) {
            var sound = sounds[i];
            if (i == 0) {
                m_chooser.setDefaultOption(sound.name, sound.file);
            } else {
                m_chooser.addOption(sound.name, sound.file);
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
