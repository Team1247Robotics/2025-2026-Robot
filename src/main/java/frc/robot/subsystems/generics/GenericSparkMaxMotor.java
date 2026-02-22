package frc.robot.subsystems.generics;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericSparkMaxMotor extends SubsystemBase {
  private final SparkMax m_motor;
  private final SparkClosedLoopController m_clController;
  private final RelativeEncoder m_encoder;

  public GenericSparkMaxMotor(int canId, MotorType motorType, SparkBaseConfig config) {
    m_motor = new SparkMax(canId, motorType);
    m_clController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  public void setEffort(double target) {
    m_motor.set(target);
  }


  /**
   * Stop the shooter motor
   */
  public void stop() {
    m_motor.set(0.0);
  }

  protected void sendStatsToDash(String name) {
    SmartDashboard.putNumber(name + " Velocity", getVelocity());
    SmartDashboard.putNumber(name + " Position", getPosition());
  }
}
