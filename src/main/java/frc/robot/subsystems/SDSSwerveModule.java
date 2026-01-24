// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;

public class SDSSwerveModule {
  private final SparkFlex m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final SparkAbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_driveClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   */
  public SDSSwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      double chassisAngularOffset,
      boolean invertDrive
    ) {
    m_driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    m_driveClosedLoopController = m_driveMotor.getClosedLoopController();
    m_turningClosedLoopController = m_turningMotor.getClosedLoopController();

    SparkFlexConfig driveConfig = Configs.SDSSwerveModule.drivingConfig;
    driveConfig.inverted(invertDrive);

    SparkMaxConfig turningConfig = Configs.SDSSwerveModule.turningConfig;
    turningConfig.absoluteEncoder.zeroOffset(chassisAngularOffset);

    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition())
      );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition())
      );
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = desiredState;
    Rotation2d currentRoation = new Rotation2d(m_turningEncoder.getPosition());

    correctedDesiredState.optimize(currentRoation);
    correctedDesiredState.cosineScale(currentRoation);

    m_driveClosedLoopController.setSetpoint(
      Math.max(
        Math.min(
          correctedDesiredState.speedMetersPerSecond,
          DriveConstants.kMaxSpeedMetersPerSecond
        ),
        -DriveConstants.kMaxSpeedMetersPerSecond
      ), ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes drive encoder. */
  public void resetEncoder() {
    m_driveEncoder.setPosition(0);
  }
}
