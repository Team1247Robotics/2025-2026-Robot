package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GetAlliance;

public class AutoBuilderSubsystem extends SubsystemBase {
  private final SendableChooser<Command> autoChooser;

  public AutoBuilderSubsystem(DriveSubsystem drivetrain) {
    
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    
    // if (config != null) {
      AutoBuilder.configure(
        drivetrain::getPose,
        drivetrain::resetOdometry,
        drivetrain::getChassisSpeeds,
        (speeds, feedForwards) -> drivetrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false),
        new PPHolonomicDriveController(
          new PIDConstants(0.5, 0, 0),
          new PIDConstants(0.5, 0, 0)
          ),
          config,
          GetAlliance::isRedAlliance,
          this
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
