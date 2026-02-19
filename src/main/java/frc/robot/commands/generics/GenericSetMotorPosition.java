package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

public class GenericSetMotorPosition extends Command {
  private final DoubleSupplier m_targetSupplier;
  protected final GenericSparkMaxMotor m_motor;
  protected final double m_allowableError = GenericConstants.MotorPositionControlAllowableError;
  private double m_target = 0;
  
  public GenericSetMotorPosition(GenericSparkMaxMotor motor, DoubleSupplier targetSupplier) {
    m_targetSupplier = targetSupplier;
    m_motor = motor;
    addRequirements(m_motor);
  }
  
  public GenericSetMotorPosition(GenericSparkMaxMotor motor, double targetPosition) {
    this(motor, () -> targetPosition);
  }

  protected double getDistanceFromTarget(double target) {
    double currentPos = m_motor.getPosition();
    return Math.abs(currentPos - target);
  }

  protected double getDistanceFromTarget() {
    return getDistanceFromTarget(m_target);
  }

  protected void moveToTarget(double target) {
    m_motor.setPosition(target);
  }

  protected void moveToTarget() {
    moveToTarget(m_target);
  }

  @Override
  public void initialize() {
    m_target = m_targetSupplier.getAsDouble();
  }

  @Override
  public void execute() {
    moveToTarget();
  }

  @Override
  public boolean isFinished() {
    return getDistanceFromTarget() < m_allowableError;
  }
}
