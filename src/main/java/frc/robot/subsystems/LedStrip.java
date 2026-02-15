package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling an LED strip.
 */
public class LedStrip extends SubsystemBase {

    /** The PWM port for the LED strip */
    public static final int LED_PORT = 0;
    /** The number of LEDs in the strip */
    public static final int LED_COUNT = 30;
    /** The density of LEDs per meter */
    public static final double LED_DENSITY_LEDS_PER_METER = 60.0;

    /** The AddressableLED object for controlling the LED strip */
    private final AddressableLED m_led;
    /** The buffer to hold the LED data */
    private final AddressableLEDBuffer m_buffer;
    /** The rainbow LED pattern */
    private final LEDPattern m_rainbowPattern;
    //**The yellow LED pattern */
    private final LEDPattern m_yellowLedPattern;

    /** Creates a new LedStrip and initializes the LED strip */
    public LedStrip() {
        m_led = new AddressableLED(LED_PORT);
        m_buffer = new AddressableLEDBuffer(LED_COUNT);

        m_led.setLength(LED_COUNT);

        m_led.start();

        m_rainbowPattern = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75),
                Meters.of(1 / LED_DENSITY_LEDS_PER_METER));

        m_yellowLedPattern = LEDPattern.gradient(GradientType.kContinuous, Color.kBlack, Color.kYellow, Color.kBlack)
                  .scrollAtAbsoluteSpeed(
                          MetersPerSecond.of(2.0),
                          Meters.of(1 / LED_DENSITY_LEDS_PER_METER)
                  );

    }

    /** Displays the rainbow pattern on the LED strip */
    public void showRainbow() {
        m_rainbowPattern.applyTo(m_buffer);

        m_led.setData(m_buffer);
    }

    public void showYellow() {
        m_yellowLedPattern.applyTo(m_buffer);

        m_led.setData(m_buffer);
    }

    /**
     * Sets the whole strip to one color
     * 
     * @param color the color to set the strip to 
     */
    public void setSolidColor(Color color) {
        LEDPattern.solid(color).applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    /** Updates the LED strip */
    public void periodic() {
        // empty
    }
}
