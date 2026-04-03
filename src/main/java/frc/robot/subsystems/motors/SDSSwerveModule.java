// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motors;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

/** This class represents a single swerve module with a drive and turning motor. */
public class SDSSwerveModule {
  public static record SDSSwerveModuleConfig(int driveCanId, int pivotCanId, Angle zeroOffset, boolean invertDrive) {}
  
  private final SparkBase m_driveMotor;
  private final SparkBase m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final SparkAbsoluteEncoder m_turnEncoder;

  private final SparkClosedLoopController m_driveClosedLoopController;
  private final SparkClosedLoopController m_turnClosedLoopController;
  
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private SparkSim m_driveMotorSim;
  private SparkSim m_turnMotorSim;
  
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   */
  @SuppressWarnings("resource") // its probably fine
  public SDSSwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      double chassisAngularOffset,
      boolean invertDrive
    ) {
    m_driveMotor = DriveConstants.createMotorController(DriveConstants.DriveControllerType, driveMotorChannel);
    m_turnMotor = DriveConstants.createMotorController(DriveConstants.TurningControllerType, turningMotorChannel);

    SmartDashboard.setDefaultNumber("Drive Motor " + driveMotorChannel + " Target Velocity", 0);
    SmartDashboard.setDefaultNumber("Drive Motor " + driveMotorChannel + " Velocity", 0);
    SmartDashboard.setDefaultNumber("Turning Motor " + turningMotorChannel + " Target Theta", 0);
    SmartDashboard.setDefaultNumber("Turning Motor " + turningMotorChannel + " Theta", 0);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getAbsoluteEncoder();

    m_driveClosedLoopController = m_driveMotor.getClosedLoopController();
    m_turnClosedLoopController = m_turnMotor.getClosedLoopController();

    SparkMaxConfig driveConfig = Configs.SDSSwerveModule.drivingConfig;
    driveConfig.inverted(invertDrive);

    SparkMaxConfig turningConfig = Configs.SDSSwerveModule.turningConfig;
    turningConfig.absoluteEncoder.zeroOffset(chassisAngularOffset);

    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());

    if (Robot.isSimulation()) {
      m_driveMotorSim = new SparkSim(m_driveMotor, DCMotor.getNEO(1));
      m_turnMotorSim = new SparkSim(m_turnMotor, DCMotor.getNEO(1));
      RobotContainer.GLOBAL_SIMULATED_BATTERY.registerPowerDrain(this::getSimulatedDrivePowerDrain);
      RobotContainer.GLOBAL_SIMULATED_BATTERY.registerPowerDrain(this::getSimulatedTurnPowerDrain);
    }

    // SmartDashboard.getNum
  }

  public double getSimulatedDrivePowerDrain() {
    return m_driveMotorSim.getMotorCurrent();
  }

  public double getSimulatedTurnPowerDrain() {
    return m_turnMotorSim.getMotorCurrent();
  }

  public SDSSwerveModule(SDSSwerveModuleConfig config) {
    this(config.driveCanId, config.pivotCanId, config.zeroOffset.in(Rotations), config.invertDrive);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveMotorVelocity(),
      getTurnMotorRotation2d()
    );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDriveMotorPosition(),
      getTurnMotorRotation2d().unaryMinus()
    );
  }

  private Rotation2d getTurnMotorRotation2d() {
    if (Robot.isSimulation()) {
      return new Rotation2d(m_turnMotorSim.getPosition());
    }
    return new Rotation2d(m_turnEncoder.getPosition());
  }

  private double getDriveMotorVelocity() {
    if (Robot.isSimulation()) {
      return m_driveMotorSim.getVelocity();
    }
    return m_driveEncoder.getVelocity();
  }

  private double getDriveMotorPosition() {
    if (Robot.isSimulation()) {
      return m_driveMotorSim.getPosition();
    }
    return m_driveEncoder.getPosition();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = desiredState;
    Rotation2d currentRotation = getTurnMotorRotation2d();

    correctedDesiredState.optimize(currentRotation);
    correctedDesiredState.cosineScale(currentRotation);

    SmartDashboard.putNumber("Drive Motor " + m_driveMotor.getDeviceId() + " Target Velocity", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Drive Motor " + m_driveMotor.getDeviceId() + " Velocity", getDriveMotorVelocity());
    SmartDashboard.putNumber("Turning Motor " + m_turnMotor.getDeviceId() + " Target Theta", desiredState.angle.getDegrees());
    SmartDashboard.putNumber("Turning Motor " + m_turnMotor.getDeviceId() + " Theta", getTurnMotorRotation2d().getDegrees());

    if (Robot.isSimulation()) {
      m_driveMotorSim.iterate(desiredState.speedMetersPerSecond, RobotContainer.GLOBAL_SIMULATED_BATTERY.getVoltage().in(Volts), DriveConstants.kDrivePeriod.in(Seconds));
      m_turnMotorSim.setPosition(desiredState.angle.getRadians());
      return;
    }

    m_driveClosedLoopController.setSetpoint(
      Math.max(
        Math.min(
          correctedDesiredState.speedMetersPerSecond,
          DriveConstants.kMaxSpeed.in(MetersPerSecond)
        ),
        -DriveConstants.kMaxSpeed.in(MetersPerSecond)
      ), ControlType.kVelocity);

    /*m_driveClosedLoopController.setSetpoint(
      correctedDesiredState.speedMetersPerSecond,
      ControlType.kVelocity);*/

    m_turnClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes drive encoder. */
  public void resetEncoder() {
    if (Robot.isSimulation()) {
      m_driveMotorSim.setPosition(0);
      return;
    }
    m_driveEncoder.setPosition(0);
  }
}
