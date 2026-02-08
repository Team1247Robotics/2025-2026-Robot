package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.utils.GetAlliance;

public class AutoBuilder2 {
  private final SendableChooser<Command> autoChooser;

  public AutoBuilder2(DriveSubsystem drivetrain) {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Error e) {
      throw new Error();
    } catch (IOException e) {
      throw new Error();
    } catch (ParseException e) {
      throw new Error();
    }
    
    // if (config != null) {
      AutoBuilder.configure(
        drivetrain::getPose,
        drivetrain::resetPose,
        drivetrain::getChassisSpeeds,
        (speeds, feedForwards) -> {
          drivetrain.drive(speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
        },
        new PPHolonomicDriveController(
          new PIDConstants(2.3, 0, 0),
          new PIDConstants(1.4, 0, 0)
        ),
        config,
        GetAlliance::isRedAlliance,
        drivetrain
        );
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    // }
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
