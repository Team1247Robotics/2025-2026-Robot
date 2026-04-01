package frc.robot.commands.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.RPM;


public class targetingCommand extends Command{
    private static final double kDistancePublishThresholdMeters = 0.01;

    private final NetworkTableEntry distEntry =
        NetworkTableInstance.getDefault()
            .getTable("vision")
            .getEntry("distanceMeters");
    private final NetworkTableEntry shooterComputeEntry =
        NetworkTableInstance.getDefault()
            .getTable("")
            .getEntry("ShooterCompute");

    private Double m_lastPublishedDistanceMeters = null;
    private double m_lastComputedRpm = ShooterConstants.kTargetSpeed.abs(RPM);
    

    public void BroadcastDist(Pose2d m_robotPos, boolean targetIsHub){
        double dist;
        final Translation2d HubPosition;
        if (targetIsHub){
            if (DriverStation.getAlliance().get() == Alliance.Red){
                HubPosition = new Translation2d(15.75, 4.10);
            }else{
                HubPosition = new Translation2d(8.25, 4.10);
            }

            dist=Math.sqrt(
                Math.pow(m_robotPos.getX() - HubPosition.getX(), 2)
                +Math.pow(m_robotPos.getY() - HubPosition.getY(), 2));

        }else {

            final double zoneX = (DriverStation.getAlliance().get() == Alliance.Red) ? 22.0:3.0;
            double maxY=16.46;
            double xDist=Math.abs(zoneX - m_robotPos.getX());

            double ang2Top = Math.atan2(maxY-(m_robotPos.getY()), xDist);
            double ang2Bott = Math.atan2(m_robotPos.getY(), xDist);

            double minAngle=Math.min(ang2Bott, ang2Top);
            double maxAngle=Math.max(ang2Bott, ang2Top);
            double robotRot = m_robotPos.getRotation().getRadians();

            if (robotRot<maxAngle && robotRot>minAngle){
                dist = xDist/Math.cos(robotRot);
            } else {
                dist = 0;
            }
        }
        if (dist > 0 && shouldPublishDistance(dist)){
            distEntry.setDouble(dist);
            m_lastPublishedDistanceMeters = dist;
        }
    }

    public double ConsumeShooterCompute() {
        double computedRpm = shooterComputeEntry.getDouble(m_lastComputedRpm);
        if (computedRpm > 0) {
            m_lastComputedRpm = computedRpm;
        }
        return m_lastComputedRpm;
    }

    private boolean shouldPublishDistance(double distanceMeters) {
        if (m_lastPublishedDistanceMeters == null) {
            return true;
        }

        return Math.abs(distanceMeters - m_lastPublishedDistanceMeters) >= kDistancePublishThresholdMeters;
    }
}
