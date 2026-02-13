// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.hub.AlwaysFaceHub;
import frc.robot.commands.ledstrip.LedStripScrollRainbow;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.AutoBuilder2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedStrip;
import frc.robot.subsystems.LonelyTalonFx;
import frc.robot.sensors.ColorSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  private final LonelyTalonFx m_badAppleMachine = new LonelyTalonFx();

  private final AutoBuilder2 m_autoBuilder = new AutoBuilder2(m_robotDrive);

  private final ColorSensor m_indexerSensor = new ColorSensor(0);

//   private final Intake m_intake = new Intake();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    new PhotonVision.PhotonVisionEstimationSubsystem(m_robotDrive::updatePoseWithPhotonVision);


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

    
    m_ledStrip.setDefaultCommand(new LedStripScrollRainbow(m_ledStrip).ignoringDisable(true));
    // m_intake.setDefaultCommand(new RunCommand(() -> {m_intake.aspire();}, m_intake));

    Trigger dpad_up = new POVButton(m_driverController, 0);
    Trigger dpad_down = new POVButton(m_driverController, 180);
    
    dpad_up.onTrue(new ResetHeading.ResetHeadingForward(m_robotDrive));
    dpad_down.onTrue(new ResetHeading.ResetHeadingBackward(m_robotDrive));

    // Trigger a_push = new Trigger(() -> m_driverController.getAButton());

    // a_push.whileTrue(new FacePointTest(m_robotDrive, m_driverController));
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

    // new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new LedStripSetGreen(m_ledStrip));

    new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(new AlwaysFaceHub(
      m_robotDrive,
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
      () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
      true
      ));

    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(Commands.runOnce(m_badAppleMachine::playBadApple, m_badAppleMachine));
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(Commands.runOnce(m_badAppleMachine::stop, m_badAppleMachine));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoBuilder.getAutonomousCommand();
  }


}
