package frc.robot.subsystems.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IndexerMotor;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(IndexerConstants.kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_clController;

  public Indexer() {
    m_encoder = m_motor.getEncoder();
    m_clController = m_motor.getClosedLoopController();
    m_motor.configure(IndexerMotor.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public void setPosition(double target) {
    m_clController.setSetpoint(target, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setVelocity(double target) {
    m_clController.setSetpoint(target, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }
}
