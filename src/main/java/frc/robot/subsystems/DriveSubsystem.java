// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  //#region Construct swerve modules
  private final SDSSwerveModule m_frontLeft = new SDSSwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontLeftDriveInverted
    );

  private final SDSSwerveModule m_rearLeft = new SDSSwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftChassisAngularOffset,
      DriveConstants.kBackLeftDriveInverted
    );

  private final SDSSwerveModule m_frontRight = new SDSSwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kFrontRightDriveInverted
    );

  private final SDSSwerveModule m_rearRight = new SDSSwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightChassisAngularOffset,
      DriveConstants.kBackRightDriveInverted
    );
  //#endregion

  private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);

  private final Field2d m_field = new Field2d();

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          Pose2d.kZero
          );

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable limelight = inst.getTable("limelight");
    DoubleArraySubscriber botPostSub = limelight
      .getDoubleArrayTopic("botpose_targetspace")
      .subscribe(new double[] {0,0,0,0,0,0});
    
    inst.addListener(
      botPostSub,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      this::updatePose
    );
  }

  /**
   * Recieve absolute position updates from limelight when it detects and apriltag.
   * This should only by called by the NetworkTables subscription.
   * 
   * @param event The event recieved from NetworkTables
   */
  private void updatePose(NetworkTableEvent event) { 
    boolean isThisAGoodIdea = false;
    if (!isThisAGoodIdea) return;

    
    double[] botpose = event.valueData.value.getDoubleArray(); // [X, Y, Z, roll, pitch, yaw]
    double x = botpose[0];
    double y = botpose[1];
    double rotation = botpose[5];

    if (x == 0 && y == 0 && rotation == 0) return;
    visionCorrectPose(new Pose2d(x, y, new Rotation2d(rotation)), Timer.getFPGATimestamp());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.updateWithTime(
        Timer.getFPGATimestamp(),
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    m_field.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putString("Angle", m_gyro.getRotation2d().toString());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Corrects the pose of the robot with an external pose provider, such as vision.
   * 
   * @param pose The pose to which to set the odometry
   */
  public void visionCorrectPose(Pose2d pose, double timestampSeconds) {
    m_odometry.addVisionMeasurement(pose, timestampSeconds);
  }

  /**
   * Abstracts selection between different methods of ChassisSpeed construction depending on field_relative
   * 
   * @param xSpeed Target x speed
   * @param ySpeed Target y speed
   * @param rotation Target rotation speed
   * @param field_relative Set if movement should be relative to the field. Regardless of the current heading of the robot, movement along the x will always move in the same direction relative to the field.
   * @return
   */
  private ChassisSpeeds createChassisSpeeds(double xSpeed, double ySpeed, double rotation, boolean field_relative) {
    if (field_relative) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed,
        ySpeed,
        rotation,
        m_gyro.getRotation2d()
      );
    }
    return new ChassisSpeeds(
      xSpeed,
      ySpeed,
      rotation
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    final ChassisSpeeds speeds = createChassisSpeeds(xSpeed, ySpeed, rot, fieldRelative);

    var swerveModuleStates = DriveConstants
      .kDriveKinematics
      .toSwerveModuleStates(
        ChassisSpeeds.discretize(
          speeds,
          DriveConstants.kDrivePeriod
        )
      );
    
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[2]);
    m_frontRight.setDesiredState(desiredStates[3]);
    m_rearLeft.setDesiredState(desiredStates[0]);
    m_rearRight.setDesiredState(desiredStates[1]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoder();
    m_rearLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_rearRight.resetEncoder();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
