// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.AlwaysFaceTag;
import frc.robot.commands.ledstrip.LedStripScrollRainbow;
import frc.robot.commands.ledstrip.LedStripSetGreen;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedStrip;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LedStrip m_ledStrip = new LedStrip();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxAngularSpeed,
                false
            ),
            m_robotDrive));

    // m_robotDrive.setDefaultCommand(new AlwaysFaceTag(m_robotDrive, m_driverController));
    
    m_ledStrip.setDefaultCommand(new LedStripScrollRainbow(m_ledStrip).ignoringDisable(true));

    Trigger dpad_up = new POVButton(m_driverController, 0);
    Trigger dpad_down = new POVButton(m_driverController, 180);
    
    dpad_up.onTrue(Commands.runOnce(() -> {
        double angle = m_robotDrive.isBlueAlliance() ? 0 : Math.PI;
        m_robotDrive.adjustGyro(angle);
        Pose2d current_pose = m_robotDrive.getPose();
        Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
        m_robotDrive.resetOdometry(pose);
    }, m_robotDrive));

    dpad_down.onTrue(Commands.runOnce(() -> {
        double angle = m_robotDrive.isBlueAlliance() ? Math.PI : 0;
        m_robotDrive.adjustGyro(angle);
        Pose2d current_pose = m_robotDrive.getPose();
        Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
        m_robotDrive.resetOdometry(pose);
    }, m_robotDrive));

    Trigger a_push = new Trigger(() -> m_driverController.getAButton());

    a_push.whileTrue(new AlwaysFaceTag(m_robotDrive, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(
            new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive
            )
        );

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(
            new InstantCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive
            )
        );

    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new LedStripSetGreen(m_ledStrip));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d.kZero,
            new ArrayList<Translation2d>(),
            new Pose2d(1, 0, Rotation2d.kZero),
            config
        );

    var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints
        );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ParallelRaceGroup swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive
        ).repeatedly().until(() -> false);

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }


}
