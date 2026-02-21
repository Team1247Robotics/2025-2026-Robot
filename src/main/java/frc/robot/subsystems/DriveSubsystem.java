// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.motors.SDSSwerveModule;
// import frc.robot.sensors.LimelightHelpers; // this is causing an incomprehensible build error that i am not dealing with rn
import frc.robot.utils.Controller;
import frc.robot.utils.GetAlliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** This class represents the robot's drive subsystem. */
public class DriveSubsystem extends SubsystemBase {


  //#region Construct swerve modules
  private final SDSSwerveModule m_frontLeft = new SDSSwerveModule(
      DriveConstants.frontLeftConfig
    );

  private final SDSSwerveModule m_rearLeft = new SDSSwerveModule(
      DriveConstants.rearLeftConfig
    );

  private final SDSSwerveModule m_frontRight = new SDSSwerveModule(
      DriveConstants.frontRightConfig
    );

  private final SDSSwerveModule m_rearRight = new SDSSwerveModule(
      DriveConstants.rearRightConfig
    );
  //#endregion

  // private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);
  private final Pigeon2 m_gyro = new Pigeon2(21);

  private final Field2d m_field = new Field2d();

  private boolean disableOdoCorrection = false;

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

  }

  public ChassisSpeeds getChassisSpeeds() {
    // ChassisSpeeds requires robot-relative speeds, so we convert the field-relative speeds from the module states using the current gyro angle.
    // BUT why are we not getting the robot-relative speeds from the module states in the first place?
    // getModuleStates() does not seem to be the proper counterpart of setModuleStates() as the order of the modules in the array is different.
    // TODO: maybe rethink the abstracted relationship between ChassisSpeeds and SwerveModuleStates in this class.
    return ChassisSpeeds.fromFieldRelativeSpeeds(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()), m_gyro.getRotation2d());
    
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
  }

  /**
   * Receive absolute position updates from limelight when it detects and AprilTag.
   * This should only by called by the NetworkTables subscription.
   * 
   * @param event The event received from NetworkTables
   */
  // private void updatePose() { 

  //   PoseEstimate pose;

  //   if (isBlueAlliance()) {
  //     pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
  //   } else {
  //     pose = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
  //   }

  //   if (pose == null) return;

  //   visionCorrectPose(pose.pose, pose.timestampSeconds);
  // }

  public void updatePoseWithPhotonVision(Pose2d pose, double timestamp) {
    visionCorrectPose(pose, timestamp);
  }

  /**
   * Check if the alliance is the blue alliance. Defaults to true if the alliance is undefined (ie. in a simulation).
   */
  public boolean isBlueAlliance() {
    return GetAlliance.isBlueAlliance();
  }

  /**
   * Inverted response of {@link #isBlueAlliance()}. Defaults to false if the alliance is undefined.
   * @return if alliance is red.
   */
  public boolean isRedAlliance() {
    return GetAlliance.isRedAlliance();
  }

  /**
   * Toggle if the camera should be used for pose correction using AprilTags
   * @return New state of odo correction.
   */
  public boolean toggleOdoCorrection() {
    disableOdoCorrection = !disableOdoCorrection;
    return disableOdoCorrection;
  }

  /**
   * Set if the camera should be used for pose correction using AprilTags
   * @param value - True for enable correction, false for off.
   * @return New state of odo correction.
   */
  public boolean setOdoCorrection(boolean value) {
    disableOdoCorrection = value;
    return disableOdoCorrection;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
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

    // updatePose();
    // postTargetToSmartDashboard();
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
  public void resetPose(Pose2d pose) {
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
   * Checks if limelight data is at least present in NetworkTables.
   * @return If "targetpose_robotspace" was found in NetworkTables.
   */
  public boolean isLimelightSafe() {
    return NetworkTableInstance.getDefault().getTable("limelight").containsKey("targetpose_robotspace");
  }

  /**
   * Sends the current target data to SmartDashboard.
   */
  // private void postTargetToSmartDashboard() {
  //   // if (!isLimelightSafe()) return;

  //   // Pose3d tagPosition3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
  //   // Pose2d tagPosition = new Pose2d(tagPosition3d.getX(), tagPosition3d.getY(), tagPosition3d.getRotation().toRotation2d());

  //   SmartDashboard.putString("Target Position Robot Space", tagPosition.toString());
  // }

  /**
   * Corrects the pose of the robot with an external pose provider, such as vision.
   * 
   * @param pose The pose to which to set the odometry
   */
  public void visionCorrectPose(Pose2d pose, double timestampSeconds) {
    if (disableOdoCorrection) return;
    final double x = pose.getX();
    final double y = pose.getY();
    if (x == 0 && y == 0) return;
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
        new Rotation2d(-m_gyro.getRotation2d().getRadians())
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

  public Command defaultControllerCommand(CommandXboxController controller) {
    return new RunCommand(() -> this.drive(
      Controller.applyDriveYFilters(controller::getLeftY),
      Controller.applyDriveXFilters(controller::getLeftX),
      Controller.applyDriveRotationFilters(controller::getRightX),
      false
    ), this);
  }

  public void drive(ChassisSpeeds speeds) {
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
   * Resets the gyro yaw. Wrapper function for {@link #zeroHeading()}.
   */
  public void zeroGyro() {
    zeroHeading();
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
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }


  /**
   * Adjusts the gyro so the set angle is zero.
   * @param angle - Double representation of of the direction in radians.
   */
  public void adjustGyro(double angle) {
    m_gyro.setYaw(angle);
  
  }
}
