// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.LedConfigs;
import frc.robot.Constants.Feature;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ToggleCommand;
import frc.robot.commands.ledstrip.LedStripScrollRainbow;
import frc.robot.commands.ledstrip.LedStripScrollYellow;
import frc.robot.commands.ledstrip.LedStripSetAlianceColor;
import frc.robot.commands.ledstrip.LedStripSetGreen;
import frc.robot.commands.motors.ClimberCommands;
import frc.robot.commands.motors.FeederCommands;
import frc.robot.commands.motors.Indexer.UpperIndexerCommands;
import frc.robot.commands.motors.Indexer.LowerIndexerCommands;
import frc.robot.commands.motors.IntakeCommands;
import frc.robot.commands.motors.ShooterCommands;
import frc.robot.commands.motors.drivetrain.HubCommands;
import frc.robot.commands.motors.drivetrain.ResetHeading;
import frc.robot.commands.targeting.targetingCommand;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.AutoBuilder2;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.LedStrip;
import frc.robot.subsystems.motors.BeltFeeder;
import frc.robot.subsystems.motors.Climber;
import frc.robot.subsystems.motors.Intake;
import frc.robot.subsystems.motors.IntakeDeployment;
import frc.robot.subsystems.motors.LonelyTalonFx;
import frc.robot.subsystems.motors.Shooter;
import frc.robot.subsystems.motors.UpperIndexer;
import frc.robot.subsystems.motors.LowerIndexer;
import frc.robot.utils.SimulatedBattery;
import frc.robot.sensors.ColorSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    Feature.Shooter,
    Feature.Targeter,
    Feature.Indexer,
    Feature.Feeder,
    //Feature.Climber,
    Feature.Intake,
  };

  private final SwerveDrivetrain m_robotDrive = new SwerveDrivetrain();

  private final LedStrip m_ledStrip = new LedStrip(LedConfigs.strip1);

  private final LonelyTalonFx m_badAppleMachine = new LonelyTalonFx();

  private AutoBuilder2 m_autoBuilder = null;

  private final ColorSensor m_indexerSensor = new ColorSensor(0);

  private final Shooter m_shooter = Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)  ? new Shooter() : null;
  private final UpperIndexer m_UpperIndexer = Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)  ? new UpperIndexer() : null;
  private final LowerIndexer m_LowerIndexer = Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)  ? new LowerIndexer() : null;

  
  private final BeltFeeder m_Feeder  = Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)   ? new BeltFeeder()  : null;
  private final Intake  m_Intake  = Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)   ? new Intake()  : null;
  private final IntakeDeployment m_IntakeDeployment = Constants.isFeatureEnabled(enabledFeatures, Feature.IntakeDeployment) ? new IntakeDeployment() : null;
  
  private final Climber m_Climber = Constants.isFeatureEnabled(enabledFeatures, Feature.Climber)  ? new Climber() : null;


  public static final SimulatedBattery GLOBAL_SIMULATED_BATTERY = new SimulatedBattery();

  CommandJoystick m_driverJoystick = new CommandJoystick(OIConstants.kDriverControllerPort);

  boolean enableCopilotController = true;
  CommandXboxController m_copilotController = enableCopilotController ? new CommandXboxController(OIConstants.kCopilotControllerPort) : null;

  public ArrayList<String> runningCommands = new ArrayList<String>();

  boolean isTargeting = false;
  boolean targetIsHub = true;
  private final targetingCommand m_targetingCommand = new targetingCommand();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().onCommandInitialize(t -> runningCommands.add(t.getName()));
    CommandScheduler.getInstance().onCommandFinish(t -> runningCommands.remove(t.getName()));

    new PhotonVision.PhotonVisionEstimationSubsystem(m_robotDrive::updatePoseWithPhotonVision);
    registerPathplannerCommands();

    m_autoBuilder = new AutoBuilder2(m_robotDrive); // Must be initialized after all commands are registered since the auto builder uses the registered commands to populate the auto chooser
    
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(m_robotDrive.defaultControllerCommand(m_driverJoystick));
    // m_robotDrive.setDefaultCommand(m_robotDrive.testVelocityCommand());

    m_ledStrip.setDefaultCommand(
        new ConditionalCommand(
          new LedStripScrollRainbow(m_ledStrip),
          new LedStripSetAlianceColor(m_ledStrip),
          DriverStation::isDisabled)
      );

    m_driverJoystick.povUp().onTrue(new ResetHeading.ResetHeadingForward(m_robotDrive));
    m_driverJoystick.povDown().onTrue(new ResetHeading.ResetHeadingBackward(m_robotDrive));

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) m_Intake.setDefaultCommand(IntakeCommands.Driver.Stop(m_Intake));
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) m_shooter.setDefaultCommand(ShooterCommands.Stop(m_shooter));
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) {
      m_UpperIndexer.setDefaultCommand(UpperIndexerCommands.Stop(m_UpperIndexer));
      m_LowerIndexer.setDefaultCommand(LowerIndexerCommands.Stop(m_LowerIndexer));
    }
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) m_Feeder.setDefaultCommand(FeederCommands.Stop(m_Feeder));
 
  }

  private ToggleCommand autonShooter = new ToggleCommand();
  private ToggleCommand autonFeeder = new ToggleCommand();
  private ToggleCommand autonIntake = new ToggleCommand();
  private static final int kAutoShootStepCount = 10;
  private static final double kTriggerActivationThreshold = 0.5;

  /**
   * Register all named commands in Pathplanner
   */
  private void registerPathplannerCommands() {
    NamedCommands.registerCommand("EnableShooter", autonShooter.enable());
    NamedCommands.registerCommand("DisableShooter", autonShooter.disable());

    NamedCommands.registerCommand("EnableFeeder", autonFeeder.enable());
    NamedCommands.registerCommand("DisableFeeder", autonFeeder.disable());

    NamedCommands.registerCommand("EnableIntake", autonIntake.enable());
    NamedCommands.registerCommand("DisableIntake", autonIntake.disable());

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) {
      autonShooter.getTrigger().whileTrue(ShooterCommands.Run.Indefinitely(m_shooter));

      NamedCommands.registerCommand("AwaitShooter", ShooterCommands.Run.Await.Passively(m_shooter));
      NamedCommands.registerCommand("AwaitShooterWarmup", ShooterCommands.Run.Await.Actively(m_shooter));
      NamedCommands.registerCommand("RunShooterIndefinitely", ShooterCommands.Run.Indefinitely(m_shooter));
      NamedCommands.registerCommand("Shoot", createAutonomousShootCommand());
    } else {
      NamedCommands.registerCommand("AwaitShooter", Commands.waitSeconds(5));
      NamedCommands.registerCommand("AwaitShooterWarmup", Commands.waitSeconds(5));
      NamedCommands.registerCommand("RunShooterIndefinitely", Commands.run(() -> {}, new Subsystem[0]));
      NamedCommands.registerCommand("Shoot", Commands.run(() -> {}, new Subsystem[0]));
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) {
      NamedCommands.registerCommand("ActivateIndex", new ParallelCommandGroup(LowerIndexerCommands.Abstracts.Step(m_LowerIndexer),UpperIndexerCommands.Abstracts.Step(m_UpperIndexer)));
      NamedCommands.registerCommand("RunIndexerNTimes", new ParallelCommandGroup(LowerIndexerCommands.Abstracts.StepNTimes(m_LowerIndexer, kAutoShootStepCount), UpperIndexerCommands.Abstracts.StepNTimes(m_UpperIndexer, kAutoShootStepCount)));
    } else {
      NamedCommands.registerCommand("ActivateIndex", Commands.waitSeconds(2));
      NamedCommands.registerCommand("RunIndexerNTimes", Commands.waitSeconds(5));
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
      autonFeeder.getTrigger().whileTrue(FeederCommands.Run(m_Feeder));

      NamedCommands.registerCommand("AwaitFeeder", FeederCommands.Await.Passively(m_Feeder));
      NamedCommands.registerCommand("RunFeederIndefinitely", createShooterFeederCommand());
    } else {
      NamedCommands.registerCommand("AwaitFeeder", Commands.waitSeconds(3));
      NamedCommands.registerCommand("RunFeederIndefinitely", Commands.run(() -> {}, new Subsystem[0]));
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Climber)) {
      NamedCommands.registerCommand("AwaitClimberRetract", ClimberCommands.Await.Retract.Actively(m_Climber));
      NamedCommands.registerCommand("AwaitClimberExtend", ClimberCommands.Await.Extend.Actively(m_Climber));
    } else {
      NamedCommands.registerCommand("AwaitClimberRetract", Commands.waitSeconds(5));
      NamedCommands.registerCommand("AwaitClimberExtend", Commands.waitSeconds(5));
    }

    NamedCommands.registerCommand("AimAtHub", HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AimAtHubIndefinitely", HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AwaitAimAtHub", HubCommands.AimAt.Await.Passively(m_robotDrive));

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) {
      autonIntake.getTrigger().whileTrue(createIntakeSequenceCommand());

      NamedCommands.registerCommand("AwaitIntake", IntakeCommands.Driver.Run.Await.Passively(m_Intake));
      NamedCommands.registerCommand("AwaitIntakeInit", IntakeCommands.Driver.Run.Await.Actively(m_Intake));
      NamedCommands.registerCommand("RunIntakeIndefinitely", createIntakeSequenceCommand());
      NamedCommands.registerCommand("DeactivateIntake", Commands.none());
    } else {
      NamedCommands.registerCommand("AwaitIntake", Commands.waitSeconds(4));
      NamedCommands.registerCommand("AwaitIntakeInit", Commands.waitSeconds(4));
      NamedCommands.registerCommand("RunIntakeIndefinitely", Commands.run(() -> {}, new Subsystem[0]));
      NamedCommands.registerCommand("DeactivateIntake", Commands.none());
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.IntakeDeployment)) {
      NamedCommands.registerCommand("AwaitIntakeDeploy", IntakeCommands.Deployment.Await.Deploy.Actively(m_IntakeDeployment));
    } else {
      NamedCommands.registerCommand("AwaitIntakeDeploy", Commands.waitSeconds(3));
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
          m_driverJoystick::getY,
          m_driverJoystick::getX,
          true).applyControllerFilters(true),
        new LedStripSetGreen(m_ledStrip)
      )
    );

    m_driverJoystick.button(5).onTrue(Commands.runOnce(m_badAppleMachine::playBadApple, m_badAppleMachine));
    m_driverJoystick.button(6).onTrue(Commands.runOnce(m_badAppleMachine::stop, m_badAppleMachine));

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) {
      m_copilotController.x().whileTrue(new ParallelCommandGroup(Commands.run(() -> m_UpperIndexer.setEffort(1), m_UpperIndexer), Commands.run(() -> m_LowerIndexer.setEffort(1), m_LowerIndexer)));
      m_copilotController.y().whileTrue(new ParallelCommandGroup(Commands.run(() -> m_UpperIndexer.setEffort(-1), m_UpperIndexer), Commands.run(() -> m_LowerIndexer.setEffort(-1), m_LowerIndexer)));
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) {
      m_driverJoystick.button(7).whileTrue(new ShooterCommands.Run.Indefinitely(m_shooter, () -> ShooterConstants.kTargetSpeed.abs(RPM)));
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Targeter)) {
      //m_driverJoystick.button(7).whileTrue(new ShooterCommands.Run.Indefinitely(m_shooter, () -> ShooterConstants.kTargetSpeed.abs(RPM)));
    }

    if (enableCopilotController) {
      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) {
        m_copilotController.leftTrigger(kTriggerActivationThreshold).whileTrue(createShooterSpoolCommand());
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter, Feature.Indexer)) {
        m_copilotController.rightTrigger(kTriggerActivationThreshold).whileTrue(createShootSequenceCommand());
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) {
        m_copilotController.a().whileTrue(createIntakeSequenceCommand());
        m_copilotController.b().whileTrue(createReverseIntakeSequenceCommand());
      }

      
    }
  }

  private Command createIntakeSequenceCommand() {
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
      return Commands.parallel(
        Commands.run(() -> m_Intake.setEffort(0.33), m_Intake),
        Commands.run(() -> m_Feeder.setEffort(0.25), m_Feeder)
      );
    }

    return Commands.run(() -> m_Intake.setEffort(0.33), m_Intake);
  }

  private Command createReverseIntakeSequenceCommand() {
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
      return Commands.parallel(
        Commands.run(() -> m_Intake.setEffort(-1.0), m_Intake),
        Commands.run(() -> m_Feeder.setEffort(-1.0), m_Feeder)
      );
    }

    return Commands.run(() -> m_Intake.setEffort(-1.0), m_Intake);
  }

  private Command createShooterSpoolCommand() {
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Targeter)) {
      return Commands.parallel(
        Commands.run(() -> m_targetingCommand.BroadcastDist(m_robotDrive.getPose(), targetIsHub)),
        ShooterCommands.Run.Indefinitely(m_shooter, m_targetingCommand::ConsumeShooterCompute)
      );
    }

    return ShooterCommands.Run.Indefinitely(m_shooter);
  }

  private Command createShooterFeederCommand() {
    return Commands.run(() -> m_Feeder.setEffort(1.0), m_Feeder);
  }

  private Command createShootSequenceCommand() {
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer, Feature.Feeder)) {
      return ShooterCommands.ShooterDependant.Parallel(
        m_shooter,
        m_targetingCommand::ConsumeShooterCompute,
        Commands.parallel(
          Commands.run(() -> m_targetingCommand.BroadcastDist(m_robotDrive.getPose(), targetIsHub)),
          createShooterFeederCommand(),
          createContinuousIndexerFeedCommand(),
          new LedStripScrollYellow(m_ledStrip)
        )
      );
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) {
      return ShooterCommands.ShooterDependant.Parallel(
        m_shooter,
        m_targetingCommand::ConsumeShooterCompute,
        Commands.parallel(
          Commands.run(() -> m_targetingCommand.BroadcastDist(m_robotDrive.getPose(), targetIsHub)),
          createContinuousIndexerFeedCommand(),
          new LedStripScrollYellow(m_ledStrip)
        )
      );
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
      return ShooterCommands.ShooterDependant.Parallel(
        m_shooter,
        m_targetingCommand::ConsumeShooterCompute,
        Commands.parallel(
          Commands.run(() -> m_targetingCommand.BroadcastDist(m_robotDrive.getPose(), targetIsHub)),
          createShooterFeederCommand(),
          new LedStripScrollYellow(m_ledStrip)
        )
      );
    }

    return Commands.parallel(
      createShooterSpoolCommand(),
      new LedStripScrollYellow(m_ledStrip)
    );
  }

  private Command createAutonomousShootCommand() {
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer, Feature.Feeder)) {
      return ShooterCommands.ShooterDependant.Sequence(
        m_shooter,
        m_targetingCommand::ConsumeShooterCompute,
        Commands.deadline(
          Commands.run(() -> m_targetingCommand.BroadcastDist(m_robotDrive.getPose(), targetIsHub)),
          createIndexedShotBurstCommand(kAutoShootStepCount),
          createShooterFeederCommand()
        )
      );
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) {
      return ShooterCommands.ShooterDependant.Sequence(
        m_shooter,
        m_targetingCommand::ConsumeShooterCompute,
        Commands.deadline(
          Commands.run(() -> m_targetingCommand.BroadcastDist(m_robotDrive.getPose(), targetIsHub)),
          createIndexedShotBurstCommand(kAutoShootStepCount)
        )
      );
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
      return ShooterCommands.ShooterDependant.Sequence(
        m_shooter,
        m_targetingCommand::ConsumeShooterCompute,
        Commands.deadline(
          Commands.run(() -> m_targetingCommand.BroadcastDist(m_robotDrive.getPose(), targetIsHub)),
          createShooterFeederCommand().withTimeout(1.5)
        )
      );
    }

    return Commands.sequence(
      ShooterCommands.Run.Await.Actively(m_shooter),
      Commands.waitSeconds(1.5)
    );
  }

  private Command createContinuousIndexerFeedCommand() {
    return Commands.repeatingSequence(createSingleIndexStepCommand());
  }

  private Command createIndexedShotBurstCommand(int steps) {
    return new ParallelCommandGroup(
      LowerIndexerCommands.Abstracts.StepNTimes(m_LowerIndexer, steps),
      UpperIndexerCommands.Abstracts.StepNTimes(m_UpperIndexer, steps)
    );
  }

  private Command createSingleIndexStepCommand() {
    return new ParallelCommandGroup(
      LowerIndexerCommands.Abstracts.StepAndPause(m_LowerIndexer),
      UpperIndexerCommands.Abstracts.StepAndPause(m_UpperIndexer)
    );
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
