// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.LedConfigs;
import frc.robot.Constants.Feature;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ledstrip.LedStripScrollRainbow;
import frc.robot.commands.ledstrip.LedStripScrollYellow;
import frc.robot.commands.ledstrip.LedStripSetAlianceColor;
import frc.robot.commands.ledstrip.LedStripSetGreen;
import frc.robot.commands.motors.ClimberCommands;
import frc.robot.commands.motors.FeederCommands;
import frc.robot.commands.motors.IndexerCommands;
import frc.robot.commands.motors.IntakeCommands;
import frc.robot.commands.motors.ShooterCommands;
import frc.robot.commands.motors.drivetrain.HubCommands;
import frc.robot.commands.motors.drivetrain.ResetHeading;
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
  /**
   * Comment out features from this array to disabled them.
   */
  private final Feature[] enabledFeatures = new Feature[] {
    // Feature.Shooter,
    // Feature.Indexer,
    // Feature.Feeder,
    // Feature.Climber,
    // Feature.Intake,
    // Feature.IntakeDeployment
  };

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final LedStrip m_ledStrip = new LedStrip(LedConfigs.strip1);

  private final LonelyTalonFx m_badAppleMachine = new LonelyTalonFx();

  private AutoBuilder2 m_autoBuilder = null;

  private final ColorSensor m_indexerSensor = new ColorSensor(0);

  private final Shooter m_shooter = Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter) ?  new Shooter() : null;
  private final Indexer m_indexer = Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer) ? new Indexer() : null;
  private final Feeder  m_Feeder  = Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder) ? new Feeder() : null;
  private final Climber m_Climber = Constants.isFeatureEnabled(enabledFeatures, Feature.Climber) ? new Climber() : null;
  private final Intake  m_Intake  = Constants.isFeatureEnabled(enabledFeatures, Feature.Intake) ? new Intake() : null;

  private final IntakeDeployment m_IntakeDeployment = Constants.isFeatureEnabled(enabledFeatures, Feature.IntakeDeployment) ? new IntakeDeployment() : null;

  // private final Intake m_intake = new Intake();

  // CommandJoystick m_driverJoystick = new CommandJoystick(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);

  boolean enableCopilotController = true;
  CommandXboxController m_copilotController = enableCopilotController ? new CommandXboxController(OIConstants.kCopilotControllerPort) : null;
  
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
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) {
      NamedCommands.registerCommand("AwaitShooterWarmup", ShooterCommands.Run.Await.Actively(m_shooter));
      NamedCommands.registerCommand("RunShooterIndefinitely", ShooterCommands.Run.Indefinitely(m_shooter));
      NamedCommands.registerCommand("Shoot", ShooterCommands.Run.Indefinitely(m_shooter));
    } else {
      NamedCommands.registerCommand("AwaitShooterWarmup", Commands.none());
      NamedCommands.registerCommand("RunShooterIndefinitely", Commands.none());
      NamedCommands.registerCommand("Shoot", Commands.none());
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) {
      NamedCommands.registerCommand("ActivateIndex", IndexerCommands.Abstracts.Step(m_indexer));
      NamedCommands.registerCommand("RunIndexerNTimes", IndexerCommands.Abstracts.StepNTimes(m_indexer, 10));
    } else {
      NamedCommands.registerCommand("ActivateIndex", Commands.none());
      NamedCommands.registerCommand("RunIndexerNTimes", Commands.none());
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
      NamedCommands.registerCommand("RunFeederIndefinitely", FeederCommands.Run(m_Feeder));
    } else {
      NamedCommands.registerCommand("RunFeederIndefinitely", Commands.none());
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Climber)) {
      NamedCommands.registerCommand("AwaitClimberRetract", ClimberCommands.Await.Retract.Actively(m_Climber));
      NamedCommands.registerCommand("AwaitClimberExtend", ClimberCommands.Await.Extend.Actively(m_Climber));
    } else {
      NamedCommands.registerCommand("AwaitClimberRetract", Commands.none());
      NamedCommands.registerCommand("AwaitClimberExtend", Commands.none());
    }

    NamedCommands.registerCommand("AimAtHub", HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AimAtHubIndefinitely", HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AwaitAimAtHub", HubCommands.AimAt.Await.Passively(m_robotDrive));

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) {
      NamedCommands.registerCommand("AwaitIntakeInit", IntakeCommands.Driver.Run.Await.Actively(m_Intake));
      NamedCommands.registerCommand("RunIntakeIndefinitely", IntakeCommands.Driver.Run.Indefinitely(m_Intake));
      NamedCommands.registerCommand("DeactivateIntake", Commands.none());
    } else {
      NamedCommands.registerCommand("AwaitIntakeInit", Commands.none());
      NamedCommands.registerCommand("RunIntakeIndefinitely", Commands.none());
      NamedCommands.registerCommand("DeactivateIntake", Commands.none());
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.IntakeDeployment)) {
      NamedCommands.registerCommand("AwaitIntakeDeploy", IntakeCommands.Deployment.Await.Deploy.Actively(m_IntakeDeployment));
    } else {
      NamedCommands.registerCommand("AwaitIntakeDeploy", Commands.none());
    }
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
          m_driverJoystick::getLeftY,
          m_driverJoystick::getLeftX,
          true).applyControllerFilters(true),
        new LedStripSetGreen(m_ledStrip)
      )
    );

    m_Joystick.button(4)
        .whileTrue(new LedStripScrollYellow(m_ledStrip));

    m_driverJoystick.button(5).onTrue(Commands.runOnce(m_badAppleMachine::playBadApple, m_badAppleMachine));
    m_driverJoystick.button(6).onTrue(Commands.runOnce(m_badAppleMachine::stop, m_badAppleMachine));

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) {
      m_driverJoystick.button(7).whileTrue(new ShooterCommands.Run.Indefinitely(m_shooter, () -> ShooterConstants.targetSpeed));
    }

    if (enableCopilotController) {
      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter, Feature.Indexer)) {
        m_copilotController.a().whileTrue(ShooterCommands.ShooterDependant.Parallel(
          m_shooter,
          Commands.repeatingSequence(IndexerCommands.Abstracts.StepAndPause(m_indexer))
        ));
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) {
        m_copilotController.b().whileTrue(IntakeCommands.Driver.Run.Indefinitely(m_Intake));
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
        m_copilotController.x().whileTrue(FeederCommands.Run(m_Feeder));
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.IntakeDeployment)) {
        m_copilotController.povUp().onTrue(IntakeCommands.Deployment.Await.Deploy.Actively(m_IntakeDeployment));
        m_copilotController.povDown().onTrue(IntakeCommands.Deployment.Await.Retract.Actively(m_IntakeDeployment));
      }
    }
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
