package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.DoubleSupplier;

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
  public static record LedStripConfig(int port, int ledCount, double ledDensity) {}

  protected final double m_densityPerMeter;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  /** The strip's followers which will have the same effects applied. */
  private ArrayList<LedStrip> m_followers = new ArrayList<LedStrip>();

  public LedStrip(LedStripConfig config) {
    m_led = new AddressableLED(config.port);
    m_led.setLength(config.ledCount);

    m_buffer = new AddressableLEDBuffer(config.ledCount);
    m_led.start();

    m_densityPerMeter = config.ledDensity;
  }

  /**
   * Gets the density per meter specified in the strip config
   * @return
   */
  public double getDensityPerMeter() {
    return m_densityPerMeter;
  }

  /**
   * Send the pattern to all followers of the current object.
   * @param pattern
   */
  private void applyPatternToFollowers(LEDPattern pattern) {
    for (var i = 0; i < m_followers.size(); i++) {
      m_followers.get(i).applyPattern(pattern);
    }
  }

  /**
   * Applies a premade pattern to the strip.
   * @param pattern
   */
  public void applyPattern(LEDPattern pattern) {
    applyPatternToFollowers(pattern);

    pattern.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  /**
   * Creates a pattern that displays a single static color along the entire length of the LED strip.
   *
   * @param color the color to display
   * @return the pattern
   */
  public void applySolid(Color color) {
    var pattern = LEDPattern.solid(color);
    applyPattern(pattern);
  }

  /**
   * Creates a pattern that displays a non-animated gradient of colors across the entire length of
   * the LED strip. Colors are evenly distributed along the full length of the LED strip. The
   * gradient type is configured with the {@code type} parameter, allowing the gradient to be either
   * continuous (no seams, good for scrolling effects) or discontinuous (a clear seam is visible,
   * but the gradient applies to the full length of the LED strip without needing to use some space
   * for wrapping).
   *
   * @param type the type of gradient (continuous or discontinuous)
   * @param colors the colors to display in the gradient
   * @return a motionless gradient pattern
   */
  public void applyGradient(GradientType type, Color... colors) {
    var pattern = LEDPattern.gradient(type, colors).scrollAtAbsoluteSpeed(
        MetersPerSecond.of(0.75),
        Meters.of(1 / getDensityPerMeter())
      );
    applyPattern(pattern);
  }

  /**
   * Creates a pattern that works as a mask layer for {@link #mask(LEDPattern)} that illuminates
   * only the portion of the LED strip corresponding with some progress. The mask pattern will start
   * from the base and set LEDs to white at a proportion equal to the progress returned by the
   * function. Some usages for this could be for displaying progress of a flywheel to its target
   * velocity, progress of a complex autonomous sequence, or the height of an elevator.
   *
   * <p>For example, creating a mask for displaying a red-to-blue gradient, starting from the red
   * end, based on where an elevator is in its range of travel.
   *
   * <pre>
   *   LEDPattern basePattern = gradient(Color.kRed, Color.kBlue);
   *   LEDPattern progressPattern =
   *     basePattern.mask(progressMaskLayer(() -> elevator.getHeight() / elevator.maxHeight());
   * </pre>
   *
   * @param progressSupplier the function to call to determine the progress. This should return
   *     values in the range [0, 1]; any values outside that range will be clamped.
   * @return the mask pattern
   */
  public void applyProgressMaskLayer(DoubleSupplier supplier) {
    var pattern = LEDPattern.progressMaskLayer(supplier).scrollAtAbsoluteSpeed(
        MetersPerSecond.of(0.75),
        Meters.of(1 / getDensityPerMeter())
      );
    applyPattern(pattern);
  }

  /**
   * Creates an LED pattern that displays a rainbow across the color wheel. The rainbow pattern will
   * stretch across the entire length of the LED strip.
   *
   * @param saturation the saturation of the HSV colors, in [0, 255]
   * @param value the value of the HSV colors, in [0, 255]
   * @return the rainbow pattern
   */
  public void applyRainbow(int saturation, int value) {
    var pattern = LEDPattern.rainbow(saturation, value).scrollAtAbsoluteSpeed(
        MetersPerSecond.of(0.75),
        Meters.of(1 / getDensityPerMeter())
      );
    applyPattern(pattern);
  }

  /**
   * Display a set of colors in steps across the length of the LED strip. No interpolation is done
   * between colors. Colors are specified by the first LED on the strip to show that color. The last
   * color in the map will be displayed all the way to the end of the strip. LEDs positioned before
   * the first specified step will be turned off (you can think of this as if there's a 0 -> black
   * step by default)
   *
   * <pre>
   *   // Display red from 0-33%, white from 33% - 67%, and blue from 67% to 100%
   *   steps(Map.of(0.00, Color.kRed, 0.33, Color.kWhite, 0.67, Color.kBlue))
   *
   *   // Half off, half on
   *   steps(Map.of(0.5, Color.kWhite))
   * </pre>
   *
   * @param steps a map of progress to the color to start displaying at that position along the LED
   *     strip
   * @return a motionless step pattern
   */
  public void applySteps(Map<? extends Number,Color> steps) {
    var pattern = LEDPattern.steps(steps).scrollAtAbsoluteSpeed(
        MetersPerSecond.of(0.75),
        Meters.of(1 / getDensityPerMeter())
      );
    applyPattern(pattern);
  }

  /**
   * Adds a follower that will automatically be given the same commands as the current strip.
   * @param strip
   */
  public void addFollowerStrip(LedStrip strip) {
    m_followers.add(strip);
  }

  /**
   * Clears all of the strip's followers.
   */
  public void clearFollowers() {
    m_followers.clear();
  }

  /**
   * Makes the strip joins the target parent's followers.
   * 
   * @param parent
   */
  public void joinParentStrip(LedStrip parent) {
    parent.addFollowerStrip(this);
  }

  /**
   * Creates a pattern that displays a single static color along the entire length of the LED strip.
   *
   * @param color the color to display
   * @return the pattern
   */
  public void setSolidColor(Color color) {
    applySolid(color);
  }
}
