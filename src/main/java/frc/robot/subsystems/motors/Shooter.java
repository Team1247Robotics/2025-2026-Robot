package frc.robot.subsystems.motors;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static final double SHOOTER_RPM = 4000;

  private final SparkMax m_motor;
  private final SparkClosedLoopController m_clController;
  private final RelativeEncoder m_encoder;

  public Shooter() {
    m_motor = new SparkMax(ShooterConstants.kMotorCanId, MotorType.kBrushless);
    m_clController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    m_motor.configure(Configs.ShooterMotor.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Get the velocity of the motor
   * @return
   */
  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  /**
   * Get the position of the motor
   * @return 
   */
  public double getPosition() {
    return m_encoder.getPosition();
  }

  /**
   * Set the velocity of the motor
   * @param target
   */
  public void setVelocity(double target) {
    m_clController.setSetpoint(target, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  /**
   * Set the position of the motor.
   * @param target
   */
  public void setPosition(double target) {
    m_clController.setSetpoint(target, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }


  /**
   * Shoot the ball out of the robot
   */
  public void shoot() {
    setVelocity(SHOOTER_RPM);
  }


  /**
   * Stop the shooter motor
   */
  public void stop() {
    setVelocity(0.0);
  }
}
