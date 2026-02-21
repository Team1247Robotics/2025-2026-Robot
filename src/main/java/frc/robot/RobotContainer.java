// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.LedConfigs;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ledstrip.LedStripScrollRainbow;
import frc.robot.commands.ledstrip.LedStripScrollYellow;
import frc.robot.commands.ledstrip.LedStripSetAlianceColor;
import frc.robot.commands.ledstrip.LedStripSetGreen;
import frc.robot.commands.motors.climber.ExtendClimber;
import frc.robot.commands.motors.climber.RetractClimber;
import frc.robot.commands.motors.drivetrain.HubCommands;
import frc.robot.commands.motors.drivetrain.ResetHeading;
import frc.robot.commands.motors.feeder.RunFeeder;
import frc.robot.commands.motors.indexer.StepIndexer;
import frc.robot.commands.motors.indexer.StepIndexerNTimes;
import frc.robot.commands.motors.intake.AwaitIntakeDeployment;
import frc.robot.commands.motors.intake.IntakeCommands;
import frc.robot.commands.motors.shooter.AwaitShooterReady;
import frc.robot.commands.motors.shooter.RunShooterIndefinitely;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.AutoBuilder2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedStrip;
import frc.robot.subsystems.motors.Climber;
import frc.robot.subsystems.motors.Feeder;
import frc.robot.subsystems.motors.Indexer;
import frc.robot.subsystems.motors.Intake;
import frc.robot.subsystems.motors.IntakeDeployment;
import frc.robot.subsystems.motors.LonelyTalonFx;
import frc.robot.subsystems.motors.Shooter;
import frc.robot.sensors.ColorSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final LedStrip m_ledStrip = new LedStrip(LedConfigs.strip1);

  private final LonelyTalonFx m_badAppleMachine = new LonelyTalonFx();

  private /*final*/ AutoBuilder2 m_autoBuilder = null; //new AutoBuilder2(m_robotDrive);

  private final ColorSensor m_indexerSensor = new ColorSensor(0);

  private final Shooter m_shooter = new Shooter();
  private final Indexer m_indexer = new Indexer();
  private final Feeder  m_Feeder  = new Feeder();
  private final Climber m_Climber = new Climber();
  private final Intake  m_Intake  = new Intake();
  private final IntakeDeployment m_IntakeDeployment = new IntakeDeployment();

  // private final Intake m_intake = new Intake();

  CommandJoystick m_driverJoystick = new CommandJoystick(OIConstants.kDriverControllerPort);
  CommandXboxController m_copilotController = new CommandXboxController(OIConstants.kCopilotControllerPort);
  
  CommandJoystick m_Joystick = new CommandJoystick(OIConstants.kSimulationJoystickPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    new PhotonVision.PhotonVisionEstimationSubsystem(m_robotDrive::updatePoseWithPhotonVision);
    registerPathplannerCommands();

    m_autoBuilder = new AutoBuilder2(m_robotDrive); // Must be initialized after all commands are registered since the auto builder uses the registered commands to populate the auto chooser
    
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(m_robotDrive.defaultControllerCommand(m_driverJoystick));

    m_ledStrip.setDefaultCommand(
        new ConditionalCommand(
          new LedStripScrollRainbow(m_ledStrip),
          new LedStripSetAlianceColor(m_ledStrip),
          DriverStation::isDisabled)
      );

    m_driverJoystick.povUp().onTrue(new ResetHeading.ResetHeadingForward(m_robotDrive));
    m_driverJoystick.povDown().onTrue(new ResetHeading.ResetHeadingBackward(m_robotDrive));
  }

  /**
   * Register all named commands in Pathplanner
   */
  private void registerPathplannerCommands() {

    NamedCommands.registerCommand("AwaitShooterWarmup", new AwaitShooterReady(m_shooter, () -> ShooterConstants.targetSpeed));
    NamedCommands.registerCommand("RunShooterIndefinitely", new RunShooterIndefinitely(m_shooter, () -> ShooterConstants.targetSpeed));
    NamedCommands.registerCommand("Shoot", new RunShooterIndefinitely(m_shooter, () -> ShooterConstants.targetSpeed));

    NamedCommands.registerCommand("ActivateIndex", new StepIndexer(m_indexer));
    NamedCommands.registerCommand("RunIndexerNTimes", new StepIndexerNTimes(m_indexer, 10));

    NamedCommands.registerCommand("RunFeederIndefinitely", new RunFeeder(m_Feeder));

    NamedCommands.registerCommand("AwaitClimberRetract", new RetractClimber(m_Climber));
    NamedCommands.registerCommand("AwaitClimberExtend", new ExtendClimber(m_Climber));

    NamedCommands.registerCommand("AimAtHub", new HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AimAtHubIndefinitely", new HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AwaitAimAtHub", new HubCommands.AimAt.Await.Passively(m_robotDrive));

    NamedCommands.registerCommand("AwaitIntakeInit", new IntakeCommands.Run.Await.Actively(m_Intake));
    NamedCommands.registerCommand("RunIntakeIndefinitely", new IntakeCommands.Run.Indefinitely(m_Intake));
    NamedCommands.registerCommand("DeactivateIntake", Commands.none()); // dont tell the motor to run and it wont run idk what you expect me to do here.
    NamedCommands.registerCommand("AwaitIntakeDeploy", new AwaitIntakeDeployment.Deploy.Actively(m_IntakeDeployment));
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

    m_driverJoystick.button(1).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    m_driverJoystick.button(2).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // new JoystickButton(m_driverController,
    // XboxController.Button.kA.value).whileTrue(new LedStripSetGreen(m_ledStrip));

    m_driverJoystick.button(3).
    whileTrue(
      Commands.parallel(
        new HubCommands.AimAt.Indefinitely(
          m_robotDrive,
          m_driverJoystick::getY,
          m_driverJoystick::getX,
          true).applyControllerFilters(true),
        new LedStripSetGreen(m_ledStrip)
      )
    );

    m_Joystick.button(4)
        .whileTrue(new LedStripScrollYellow(m_ledStrip));

    m_driverJoystick.button(5).onTrue(Commands.runOnce(m_badAppleMachine::playBadApple, m_badAppleMachine));
    m_driverJoystick.button(6).onTrue(Commands.runOnce(m_badAppleMachine::stop, m_badAppleMachine));

    m_driverJoystick.button(7).whileTrue(new RunShooterIndefinitely(m_shooter, () -> ShooterConstants.targetSpeed));
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
