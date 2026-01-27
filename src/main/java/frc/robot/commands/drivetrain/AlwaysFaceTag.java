package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.sensors.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AlwaysFaceTag extends Command {
    private DriveSubsystem m_drivetrain;
    private Pose2d m_tagPosition = Pose2d.kZero;
    private XboxController m_controller;

    private Field2d m_tagOnField = new Field2d();

    /**
     * 
     * @param drivetrain
     * @param controller
     */
    public AlwaysFaceTag(DriveSubsystem drivetrain, XboxController controller) {
        m_drivetrain = drivetrain;
        m_controller = controller;
        m_tagOnField.setRobotPose(m_tagPosition);
    }

    @Override
    public void execute() {
        Pose3d tagPosition3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
        Pose2d robotPose = m_drivetrain.getPose();
        Pose2d tagPosition = new Pose2d(tagPosition3d.getY(), tagPosition3d.getX(), tagPosition3d.getRotation().toRotation2d().minus(robotPose.getRotation()));

        if (!(tagPosition.getX() == 0 && tagPosition.getY() == 0)) {
            Pose2d tagRotated = tagPosition.rotateBy(robotPose.getRotation()).rotateBy(Rotation2d.kPi);

            m_tagPosition = new Pose2d(
                robotPose.getX() + tagRotated.getX(),
                robotPose.getY() + tagRotated.getY(),
                tagRotated.getRotation()
            );
        }
        m_tagOnField.setRobotPose(m_tagPosition);
        SmartDashboard.putData("Target", m_tagOnField);

        double control_y = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
        double control_x = MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
        double turn = -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxAngularSpeed;

        Rotation2d diffRotation;
        if (m_tagPosition.getX() == 0 && m_tagPosition.getY() == 0) {
            diffRotation = new Rotation2d();
        } else {
            // diffRotation = robotPose.minus(m_tagPosition).getRotation().minus(robotPose.getRotation());
            // diffRotation = new Rotation2d(
            //     Math.atan2(-(tagPosition.getY() - robotPose.getY()), -(tagPosition.getX() - robotPose.getX()))
            // );
            Translation2d tagPoint = m_tagPosition.getTranslation();
            Rotation2d targetRotation = tagPoint.minus(robotPose.getTranslation()).getAngle();
            Rotation2d heading = targetRotation.minus(robotPose.getRotation());
            diffRotation = heading;
        }
        SmartDashboard.putString("Target Angle", diffRotation.toString());
        // double tx = (LimelightHelpers.getTX("limelight") / 360) * (Math.PI * 2);

        // m_drivetrain.drive(control_y, control_x, turn, true);
        m_drivetrain.drive(control_y, control_x, diffRotation.getRadians(), true);
    }

    public Pose2d updateTagPosition(Pose2d newTagPosition) {
        m_tagPosition = newTagPosition;
        return newTagPosition;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
