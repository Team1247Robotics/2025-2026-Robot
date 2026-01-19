package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

    /**
     * Subsystem for controlling an LED strip.
     */
public class LedStrip extends SubsystemBase {

    /** The PWM port for the LED strip */
    public static final int LED_PORT = 9;
    /** The number of LEDs in the strip */
    public static final int LED_COUNT = 30;
    /** The density of LEDs per metre */
    public static final double LED_DENSITY_LEDS_PER_METER = 1.0 / 60.0;

    /** The AddressableLED object for controlling the LED strip */
    private final AddressableLED m_led;
    /** The buffer to hold the LED dsta */
    private final AddressableLEDBuffer m_buffer;
    /** The rainbow LED pattern */
    private final LEDPattern m_rainbowPattern;

    /** Creates a new LedStrip and initializes the LED strip */
    public LedStrip() {
        m_led = new AddressableLED(LED_PORT);
        m_buffer = new AddressableLEDBuffer(LED_COUNT);

        m_led.setLength(LED_COUNT);

        m_led.start();

        m_rainbowPattern = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of( LED_DENSITY_LEDS_PER_METER ));

    }
    
    /** Displays the rainbow pattern on the LED strip */
    public void showRainbow() {
        m_rainbowPattern.applyTo(m_buffer);

        m_led.setData(m_buffer);
    }

    /** Updates the LED strip */
    public void periodic() {
    showRainbow();
    }
}
