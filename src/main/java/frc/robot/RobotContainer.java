// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LedConfigs;
import frc.robot.Constants.Feature;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ToggleCommand;
import frc.robot.commands.ledstrip.*;
import frc.robot.commands.motors.ClimberCommands;
import frc.robot.commands.motors.FeederCommands;
import frc.robot.commands.motors.IndexerCommands;
import frc.robot.commands.motors.IntakeCommands;
import frc.robot.commands.motors.ShooterCommands;
import frc.robot.commands.motors.drivetrain.*;
import frc.robot.commands.targeting.targetingCommand;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.AutoBuilder2;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.LedStrip;
import frc.robot.subsystems.motors.Climber;
import frc.robot.subsystems.motors.Feeder;
import frc.robot.subsystems.motors.Indexer;
import frc.robot.subsystems.motors.Intake;
import frc.robot.subsystems.motors.IntakeDeployment;
import frc.robot.subsystems.motors.LonelyTalonFx;
import frc.robot.subsystems.motors.Shooter;
import frc.robot.subsystems.motors.ShooterFollower;
import frc.robot.utils.SimulatedBattery;
import frc.robot.sensors.ColorSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  /**
   * Comment out features from this array to disabled them.
   */
  private final Feature[] enabledFeatures = new Feature[] {
    Feature.Shooter,
    Feature.Indexer,
    //Feature.Feeder,
    // Feature.Climber,
    Feature.Intake
    // Feature.IntakeDeployment
  };

  private final SwerveDrivetrain m_robotDrive = new SwerveDrivetrain();

  private final LedStrip m_ledStrip = new LedStrip(LedConfigs.strip1);

  //private final LonelyTalonFx m_badAppleMachine = new LonelyTalonFx();

  private AutoBuilder2 m_autoBuilder = null;

  private final ColorSensor m_indexerSensor = new ColorSensor(0);

  private final Shooter m_shooter = Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter) ?  new Shooter() : null;
  private final ShooterFollower m_shooterFollower = Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter) ?  new ShooterFollower() : null;

  private final Indexer m_indexer = Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer) ? new Indexer() : null;
  private final Feeder  m_Feeder  = Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder) ? new Feeder() : null;
  private final Climber m_Climber = Constants.isFeatureEnabled(enabledFeatures, Feature.Climber) ? new Climber() : null;
  private final Intake  m_Intake  = Constants.isFeatureEnabled(enabledFeatures, Feature.Intake) ? new Intake() : null;

  private final IntakeDeployment m_IntakeDeployment = Constants.isFeatureEnabled(enabledFeatures, Feature.IntakeDeployment) ? new IntakeDeployment() : null;

  private static final Boolean enablePilotXbox = false;

  private final CommandXboxController m_driverXbox = new CommandXboxController(OIConstants.kDriverControllerPort);

  private final CommandJoystick m_driverJoystick = new CommandJoystick(OIConstants.kDriverControllerPort);

  private Trigger driverButton(int button) {
  return enablePilotXbox
      ? m_driverXbox.button(button)
      : m_driverJoystick.button(button);
}

private Trigger driverPovUp() {
  return enablePilotXbox ? m_driverXbox.povUp() : m_driverJoystick.button(19);
}

private Trigger driverPovDown() {
  return enablePilotXbox ? m_driverXbox.povDown() : m_driverJoystick.button(20);
}
private double getForward() {
  return enablePilotXbox
      ? -m_driverXbox.getLeftY()
      : -m_driverJoystick.getY();
}

private double getStrafe() {
  return enablePilotXbox
      ? -m_driverXbox.getLeftX()
      : -m_driverJoystick.getX();
}

private double getTurn() {
  return enablePilotXbox
      ? -m_driverXbox.getRightX()
      : -m_driverJoystick.getZ(); // twist
}
  public static final SimulatedBattery GLOBAL_SIMULATED_BATTERY = new SimulatedBattery();
  // private final Intake m_intake = new Intake();

  // CommandJoystick m_driverJoystick = new CommandJoystick(OIConstants.kDriverControllerPort);
  //CommandXboxController m_driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);

  boolean enableCopilotController = true;
  CommandXboxController m_copilotController = enableCopilotController ? new CommandXboxController(OIConstants.kCopilotControllerPort) : null;
  
  //CommandJoystick m_Joystick = new CommandJoystick(OIConstants.kSimulationJoystickPort);

  private PhotonVision.PhotonVisionEstimationSubsystem pvision = null;
  private final targetingCommand m_targetingCommand = new targetingCommand();

  public ArrayList<String> runningCommands = new ArrayList<String>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandScheduler.getInstance().onCommandInitialize(t -> runningCommands.add(t.getName()));
    CommandScheduler.getInstance().onCommandFinish(t -> runningCommands.remove(t.getName()));

    pvision = new PhotonVision.PhotonVisionEstimationSubsystem(m_robotDrive::updatePoseWithPhotonVision); // The photon vision subsystem that estimates the robot's pose using the camera and updates the robot's pose in the drivetrain subsystem.

    registerPathplannerCommands(); // Registers commands with PathPlanner so that they can be used in the auto builder.

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

    driverPovUp().onTrue(new ResetHeading.ResetHeadingForward(m_robotDrive));
    driverPovDown().onTrue(new ResetHeading.ResetHeadingBackward(m_robotDrive));

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) m_Intake.setDefaultCommand(IntakeCommands.Driver.Stop(m_Intake));
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) m_shooter.setDefaultCommand(ShooterCommands.Stop(m_shooter));
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) m_indexer.setDefaultCommand(IndexerCommands.Stop(m_indexer));
    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) m_Feeder.setDefaultCommand(FeederCommands.Stop(m_Feeder));
  }
  //Add gversion to dashboard (BuildConstants.java)
  private ToggleCommand autonShooter = new ToggleCommand();
  private ToggleCommand autonFeeder = new ToggleCommand();
  private ToggleCommand autonIntake = new ToggleCommand();

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
      NamedCommands.registerCommand("ActivateIndex", IndexerCommands.Abstracts.Step(m_indexer));
      //NamedCommands.registerCommand("RunIndexerNTimes", IndexerCommands.Abstracts.StepNTimes(m_indexer, 10));
      NamedCommands.registerCommand("RunIndexerNTimes", Commands.sequence(Commands.deadline(new IndexerCommands.Abstracts.StepNTimes(m_indexer, 10), autonIntake.enable()),autonIntake.disable()));
    } else {
      NamedCommands.registerCommand("ActivateIndex", Commands.waitSeconds(2));
      NamedCommands.registerCommand("RunIndexerNTimes", Commands.waitSeconds(5));
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
      autonFeeder.getTrigger().whileTrue(FeederCommands.Run(m_Feeder));

      NamedCommands.registerCommand("AwaitFeeder", FeederCommands.Await.Passively(m_Feeder));
      NamedCommands.registerCommand("RunFeederIndefinitely", FeederCommands.Run(m_Feeder));
    } else {
      NamedCommands.registerCommand("AwaitFeeder", Commands.waitSeconds(3));
      NamedCommands.registerCommand("RunFeederIndefinitely", Commands.run(() -> {}, new Subsystem[0]));
    }

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Climber)) {
      NamedCommands.registerCommand("AwaitClimberRetract", ClimberCommands.Await.Retract.Actively(m_Climber));
      NamedCommands.registerCommand("AwaitClimberExtend", ClimberCommands.Await.Extend.Actively(m_Climber));
    } else {
      NamedCommands.registerCommand("AwaitClimberRetract", Commands.waitSeconds(1));
      NamedCommands.registerCommand("AwaitClimberExtend", Commands.waitSeconds(1));
    }

    /*NamedCommands.registerCommand("AimAtHub", HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AimAtHubIndefinitely", HubCommands.AimAt.Indefinitely(m_robotDrive));
    NamedCommands.registerCommand("AwaitAimAtHub", HubCommands.AimAt.Await.Passively(m_robotDrive));*/

    NamedCommands.registerCommand("AimAtHub", Commands.waitSeconds(1));
    NamedCommands.registerCommand("AimAtHubIndefinitely", Commands.waitSeconds(5)); // infinity is 5 seconds in auton
    NamedCommands.registerCommand("AwaitAimAtHub", Commands.waitSeconds(1));


    NamedCommands.registerCommand("AwaitAimAtHub2D", new TimedTurnUsingAprilTagCamera(m_robotDrive, pvision, 3.0)); // This command is intended to be used in auton to turn the robot towards the target before or while shooting

    if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) {
      autonIntake.getTrigger().whileTrue(IntakeCommands.Driver.Run.Indefinitely(m_Intake));

      NamedCommands.registerCommand("AwaitIntake", IntakeCommands.Driver.Run.Await.Passively(m_Intake));
      NamedCommands.registerCommand("AwaitIntakeInit", IntakeCommands.Driver.Run.Await.Actively(m_Intake));
      NamedCommands.registerCommand("RunIntakeIndefinitely", IntakeCommands.Driver.Run.Indefinitely(m_Intake));
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
      NamedCommands.registerCommand("AwaitIntakeDeploy", Commands.waitSeconds(1));
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
      boolean singleDriver = true;

    /* Driver Controller bindings */
    m_driverJoystick.button(20).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    m_driverJoystick.button(19).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    m_driverJoystick.button(31).
      whileTrue(
        Commands.parallel(
          /*new HubCommands.AimAt.Indefinitely(
            m_robotDrive,
            m_driverJoystick::getLeftY,
            m_driverJoystick::getLeftX,
            true).applyControllerFilters(true), */
          new DriveUsingAprilTagCamera(m_robotDrive, pvision, m_driverJoystick),
          //new LedStripSetGreen(m_ledStrip)
          new LedStripIndicateUsingCamera(m_ledStrip, pvision)
        )
      );

    m_driverJoystick.button(23)
      //.whileTrue(new LedStripScrollYellow(m_ledStrip));
      .onTrue(
        Commands.race(  
          new TimedTurnUsingAprilTagCamera(m_robotDrive, pvision , 20.0),
          new LedStripIndicateUsingCamera(m_ledStrip, pvision)
        )
      );

      m_driverJoystick.button(22).onTrue(new AbortTurn(m_robotDrive)); // AbortTurn command, which is intended to be used to stop the TurnUsingAprilTagCamera command when the driver wants to take control of the robot again

      /* Emergency driver overrides */
      if (singleDriver) {
        m_driverJoystick.button(1).onTrue(Commands.run(() -> m_shooter.setEffort(1), m_shooter));
        m_driverJoystick.button(2).onTrue(Commands.run(() -> m_shooter.setEffort(-1), m_shooter));
        /* Ingest Mode - Indexer + Intake at 50-75% */
        m_driverJoystick.button(14).whileTrue(Commands.parallel(Commands.run(() -> m_Intake.setEffort(.50), m_Intake),Commands.run(() -> m_indexer.setEffort(.75), m_indexer)));

        /* Spew Mode - Indexer + Intake at -50-75%*/
        m_driverJoystick.button(16).whileTrue(Commands.parallel(Commands.run(() -> m_Intake.setEffort(-.50), m_Intake), Commands.run(() -> m_indexer.setEffort(-.75), m_indexer)));

        /* Shoot Mode - run indexer at full, intake at -25*/
        m_driverJoystick.button(15).whileTrue(Commands.parallel(Commands.run(() -> m_Intake.setEffort(.50), m_Intake), Commands.run(() -> m_indexer.setEffort(-.75), m_indexer)));

        
        if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) {
          m_driverJoystick.button(6).whileTrue(ShooterCommands.Run.Indefinitely(m_shooter));
        }

      }


    /* Copilot Bindings */

    if (enableCopilotController) {
      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Indexer)) {
        m_copilotController.x().whileTrue(
          Commands.run(
            () -> m_indexer.setEffort(Constants.IndexerConstants.Control.kManualJogEffort),
            m_indexer
          )
        );
        m_copilotController.y().whileTrue(
          Commands.run(
            () -> m_indexer.setEffort(-Constants.IndexerConstants.Control.kManualJogEffort),
            m_indexer
          )
        );
      } 
      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter)) {
        m_copilotController.leftTrigger(0.5).whileTrue(createShooterSpoolCommand());
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Shooter, Feature.Indexer)) {
        m_copilotController.rightTrigger(0.5).whileTrue(createShootSequenceCommand());
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.Intake)) {
        m_copilotController.a().whileTrue(createIntakeSequenceCommand());
        m_copilotController.b().whileTrue(createReverseIntakeSequenceCommand());
      }

     /* if (Constants.isFeatureEnabled(enabledFeatures, Feature.Feeder)) {
        m_copilotController.button(4).whileTrue(FeederCommands.Run(m_Feeder));
      }

      if (Constants.isFeatureEnabled(enabledFeatures, Feature.IntakeDeployment)) {
        m_copilotController.povUp().onTrue(IntakeCommands.Deployment.Await.Deploy.Actively(m_IntakeDeployment));
        m_copilotController.povDown().onTrue(IntakeCommands.Deployment.Await.Retract.Actively(m_IntakeDeployment));
      }*/
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

  private Command createIntakeSequenceCommand() {
    return Commands.parallel(
      Commands.run(() -> m_Intake.setEffort(0.50), m_Intake),
      Commands.run(() -> m_indexer.setEffort(0.75), m_indexer)
    );
  }

  private Command createReverseIntakeSequenceCommand() {
    return Commands.parallel(
      Commands.run(() -> m_Intake.setEffort(-0.50), m_Intake),
      Commands.run(() -> m_indexer.setEffort(-0.75), m_indexer)
    );
  }

  private Command createShooterSpoolCommand() {
    return Commands.parallel(
      Commands.run(this::updateShotSolution),
      ShooterCommands.Run.Indefinitely(m_shooter, m_targetingCommand::ConsumeShooterCompute)
    );
  }

  private Command createShootSequenceCommand() {
    return Commands.parallel(
      Commands.run(this::updateShotSolution),
      Commands.either(
        Commands.repeatingSequence(IndexerCommands.Abstracts.StepAndPause(m_indexer)),
        Commands.none(),
        this::isShooterReadyToFeed
      ),
      new LedStripScrollYellow(m_ledStrip)
    );
  }

  private Command createAutonomousShootCommand() {
    return ShooterCommands.ShooterDependant.Sequence(
      m_shooter,
      m_targetingCommand::ConsumeShooterCompute,
      Commands.deadline(
        Commands.run(this::updateShotSolution),
        new IndexerCommands.Abstracts.StepNTimes(m_indexer, 10)
      )
    );
  }

  private void updateShotSolution() {
    m_targetingCommand.BroadcastShotSolution(m_robotDrive.getPose(), pvision.getLatestTargetId());
  }

  private boolean isShooterReadyToFeed() {
    double targetRpm = m_targetingCommand.ConsumeShooterCompute();
    double velocityError = Math.abs(m_shooter.getVelocity() - targetRpm);
    return velocityError <= ShooterConstants.Control.kAllowableError.abs(RPM);
  }

  public void periodic() {
    m_indexerSensor.periodic();
    updateShotSolution();

    double angle = pvision.getLatestAngleToTarget();
    double area = pvision.getLatestAreaOfTarget();

    SmartDashboard.putNumber("AngleToTarget", angle);
    SmartDashboard.putNumber("AreaOfTarget", area);

    boolean goodAngle = false;
    
    if (angle != 0.0 && Math.abs(angle) < 5.0) goodAngle = true;

    boolean goodArea = false;
    
    if (area > 0.15 && area < 0.22) goodArea = true; 

    SmartDashboard.putBoolean("GoodAngleToTarget", goodAngle);
    SmartDashboard.putBoolean("GoodAreaOfTarget", goodArea);
  }

}
