package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.utils.GetAlliance;

/** This class sets up the AutoBuilder and the auto chooser for selecting autonomous routines. */
public class AutoBuilder2 {
  private /*final*/ SendableChooser<Command> autoChooser;

  public AutoBuilder2(SwerveDrivetrain drivetrain) {
    RobotConfig config = null;

    try {
      config = RobotConfig.fromGUISettings();
    
      if (config != null) {
        AutoBuilder.configure(
          drivetrain::getPose, // Robot pose supplier
          drivetrain::resetPose, // Method to reset odometry (will be called if auto has a starting pose)
          drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedForwards) -> {
            drivetrain.drive(speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
          },
          new PPHolonomicDriveController(
            new PIDConstants(3, 0, 0),
            new PIDConstants(2, 0, 0)
          ),
          config, // The robot configuration
          GetAlliance::isRedAlliance, // Boolean supplier that controls when the path will be mirrored for the red alliance
          drivetrain // Reference to drivetrain to set requirements
          );

        autoChooser = AutoBuilder.buildAutoChooser(); // Named commands must be registered before this is called. Alternatively, those two lines could be moved into RobotContainer after all commands are registered.
        SmartDashboard.putData("Auto Chooser", autoChooser);
      }
      else
      {
        autoChooser = null;
      }

    } catch (Exception ex) {
      autoChooser = null;
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
