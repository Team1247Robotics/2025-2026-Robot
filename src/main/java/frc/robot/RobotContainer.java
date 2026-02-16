// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.AlwaysFaceHub;
import frc.robot.commands.drivetrain.ResetHeading;
import frc.robot.commands.ledstrip.LedStripScrollRainbow;
import frc.robot.commands.ledstrip.LedStripScrollYellow;
import frc.robot.commands.ledstrip.LedStripSetAlianceColor;
import frc.robot.commands.ledstrip.LedStripSetGreen;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.AutoBuilder2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedStrip;
import frc.robot.subsystems.LonelyTalonFx;
import frc.robot.sensors.ColorSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  private /*final*/ AutoBuilder2 m_autoBuilder = null; //new AutoBuilder2(m_robotDrive);

  private final ColorSensor m_indexerSensor = new ColorSensor(0);

  // private final Intake m_intake = new Intake();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  
  CommandJoystick m_Joystick = new CommandJoystick(OIConstants.kSimulationJoystickPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    new PhotonVision.PhotonVisionEstimationSubsystem(m_robotDrive::updatePoseWithPhotonVision);

    NamedCommands.registerCommand("RampUpShooter", new WaitCommand(1));  // TODO: replace with actual command
    NamedCommands.registerCommand("ActivateIndex", new WaitCommand(1)); // TODO: replace with actual command
    NamedCommands.registerCommand("Shoot", new WaitCommand(1)); // TODO: replace with actual command
    NamedCommands.registerCommand("Climb", new WaitCommand(1)); // TODO: replace with actual command
    NamedCommands.registerCommand("AcquireTarget", new WaitCommand(1)); // TODO: replace with actual command
    NamedCommands.registerCommand("ActivateIntake", new WaitCommand(1)); // TODO: replace with actual command
    NamedCommands.registerCommand("CollectIntake", new WaitCommand(1)); // TODO: replace with actual command
    NamedCommands.registerCommand("DeactivateIntake", new WaitCommand(1)); // TODO: replace with actual command

    m_autoBuilder = new AutoBuilder2(m_robotDrive); // Must be initialized after all commands are registered since the auto builder uses the registered commands to populate the auto chooser
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)
                    * DriveConstants.kMaxAngularSpeed,
                false),
            m_robotDrive));

    m_ledStrip.setDefaultCommand(
        new ConditionalCommand(
          new LedStripScrollRainbow(m_ledStrip).ignoringDisable(true),
          new LedStripSetAlianceColor(m_ledStrip).ignoringDisable(true),
          DriverStation::isDisabled)
      );


    /* 
    m_ledStrip.setDefaultCommand(
        // Default when enabled is Smart Aliance Color Thing
        // Default when disabled is Rainbow

        new RunCommand(
            () -> {
              if (DriverStation.isDisabled()) {
                  new LedStripScrollRainbow(m_ledStrip).ignoringDisable(true);
              } else {
                  new LedStripSetAlianceColor(m_ledStrip).ignoringDisable(true);
              }
        }, m_ledStrip).ignoringDisable(true)
    );
    */


    // m_intake.setDefaultCommand(new RunCommand(() -> {m_intake.aspire();},
    // m_intake));

    Trigger dpad_up = m_driverController.povUp();
    Trigger dpad_down = m_driverController.povDown();

    dpad_up.onTrue(new ResetHeading.ResetHeadingForward(m_robotDrive));
    dpad_down.onTrue(new ResetHeading.ResetHeadingBackward(m_robotDrive));

    // Trigger a_push = new Trigger(() -> m_driverController.getAButton());

    // a_push.whileTrue(new FacePointTest(m_robotDrive, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // new JoystickButton(m_driverController,
    // XboxController.Button.kA.value).whileTrue(new LedStripSetGreen(m_ledStrip));

    m_driverController.b().
    whileTrue(
      new ParallelCommandGroup(
        new AlwaysFaceHub(
          m_robotDrive,
          m_driverController::getLeftY,
          m_driverController::getLeftX,
          true).applyControllerFilters(true),
          new LedStripSetGreen(m_ledStrip).ignoringDisable(true)
          ));

    m_Joystick.button(2)
        .whileTrue(new LedStripScrollYellow(m_ledStrip).ignoringDisable(true));

    m_driverController.y()
        .onTrue(Commands.runOnce(m_badAppleMachine::playBadApple, m_badAppleMachine));
    m_driverController.x()
        .onTrue(Commands.runOnce(m_badAppleMachine::stop, m_badAppleMachine));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoBuilder.getAutonomousCommand();
  }

  public void periodic() {
    m_indexerSensor.periodic();
  }

}
