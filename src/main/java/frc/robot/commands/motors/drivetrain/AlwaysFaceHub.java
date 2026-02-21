package frc.robot.commands.motors.drivetrain;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.HubPositions;

/**
 * @deprecated
 */
public class AlwaysFaceHub extends FaceTarget2d {
  public AlwaysFaceHub(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
    super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
    m_fieldRelative = fieldRelative;
  }
}
