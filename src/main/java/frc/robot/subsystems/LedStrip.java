package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {

    public static final int ledPort = 9;
    public static final int ledCount = 30;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final LEDPattern m_rainbowPattern;

    public LedStrip() {
        m_led = new AddressableLED(ledPort);
        m_buffer = new AddressableLEDBuffer(ledCount);

        m_led.setLength(ledCount);

        m_led.start();

        m_rainbowPattern = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1.0 / ledCount));

    }
    
    public void showRainbow() {
        m_rainbowPattern.applyTo(m_buffer);

        m_led.setData(m_buffer);
    }

    public void periodic() {
    showRainbow();
    }
}
