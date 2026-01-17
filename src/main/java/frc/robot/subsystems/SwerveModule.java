// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final SparkFlex m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final SparkClosedLoopController m_driveCLController;
  private final SparkAbsoluteEncoder m_turningEncoder;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          ModuleConstants.kIModuleTurningController,
          ModuleConstants.kDriveEncoderDistancePerPulse,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel) {
    m_driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveCLController = m_driveMotor.getClosedLoopController();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    desiredState.optimize(encoderRotation);
    desiredState.cosineScale(encoderRotation);

    final double driveOutput = desiredState.speedMetersPerSecond;

    final double turnOutput = m_turningPIDController
                              .calculate(
                                m_turningEncoder
                                .getPosition(),
                                desiredState
                                .angle
                                .getRadians()
                              );


    m_driveCLController.setSetpoint(driveOutput, ControlType.kVelocity); // Using spark internal motor controller
    m_turningMotor.set(-turnOutput);
  }

  /** Zeroes drive encoder. */
  public void resetEncoder() {
    m_driveEncoder.setPosition(0);
  }
}
