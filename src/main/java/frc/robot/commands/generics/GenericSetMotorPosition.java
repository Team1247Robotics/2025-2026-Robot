package frc.robot.commands.generics;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.GenericConstants;
import frc.robot.subsystems.generics.GenericSparkMaxMotor;

public class GenericSetMotorPosition extends GenericAwaitBaseTargetWithinError {
  protected final GenericSparkMaxMotor m_motor;
  private double m_target = 0;
  
  public GenericSetMotorPosition(GenericSparkMaxMotor motor, DoubleSupplier targetSupplier) {
    super(motor::getPosition, targetSupplier, GenericConstants.MotorPositionControlAllowableError);
    setUseStaticTarget(true);
    m_motor = motor;
    addRequirements(m_motor);
  }
  
  public GenericSetMotorPosition(GenericSparkMaxMotor motor, double targetPosition) {
    this(motor, () -> targetPosition);
  }

  protected void moveToTarget(double target) {
    m_motor.setPosition(target);
  }

  protected void moveToTarget() {
    moveToTarget(m_target);
  }

  @Override
  public void execute() {
    moveToTarget();
  }
}
