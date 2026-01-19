package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {

    public static final int LED_PORT = 9;
    public static final int LED_COUNT = 30;
    public static final double LED_DENSITY_LEDS_PER_METER = 1.0 / 60.0;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final LEDPattern m_rainbowPattern;

    public LedStrip() {
        m_led = new AddressableLED(LED_PORT);
        m_buffer = new AddressableLEDBuffer(LED_COUNT);

        m_led.setLength(LED_COUNT);

        m_led.start();

        m_rainbowPattern = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of( LED_DENSITY_LEDS_PER_METER ));

    }
    
    public void showRainbow() {
        m_rainbowPattern.applyTo(m_buffer);

        m_led.setData(m_buffer);
    }

    public void periodic() {
    showRainbow();
    }
}
