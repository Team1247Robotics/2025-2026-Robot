package frc.robot.commands.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class targetingCommand extends Command{
    private final NetworkTableEntry distEntry =
        NetworkTableInstance.getDefault()
            .getTable("vision")
            .getEntry("distanceMeters");
    
    

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
        if (dist>0){
            distEntry.setDouble(dist);
        }
    }
}
