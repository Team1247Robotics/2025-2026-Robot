package frc.robot.commands.hub;

import java.util.function.DoubleSupplier;

import frc.robot.commands.drivetrain.FaceTarget2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.HubPositions;

public class AlwaysFaceHub extends FaceTarget2d {
  
  public AlwaysFaceHub(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Boolean fieldRelative) {
    super(drivetrain, HubPositions::getAllianceHubPose, xSupplier, ySupplier);
    m_fieldRelative = fieldRelative;
  }
}
