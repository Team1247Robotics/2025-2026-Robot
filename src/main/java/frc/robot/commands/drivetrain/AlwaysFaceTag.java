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
        Pose2d tagPosition = new Pose2d(tagPosition3d.getY(), tagPosition3d.getX(), tagPosition3d.getRotation().toRotation2d());
        Pose2d currentPose = m_drivetrain.getPose();

        if (!(tagPosition.getX() == 0 && tagPosition.getY() == 0)) {
            Pose2d tagRotated = tagPosition.rotateBy(currentPose.getRotation()).rotateBy(Rotation2d.kPi);

            m_tagPosition = new Pose2d(
                currentPose.getX() + tagRotated.getX(),
                currentPose.getY() + tagRotated.getY(),
                tagRotated.getRotation()
            );
        }
        // SmartDashboard.putString("position", tagPosition.toString());
        m_tagOnField.setRobotPose(m_tagPosition);
        SmartDashboard.putData("Target", m_tagOnField);

        double control_y = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
        double control_x = MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
        double yaw = -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxAngularSpeed;

        Pose2d current_pose = m_drivetrain.getPose();

        Rotation2d diff_rotation;
        if (m_tagPosition.getX() == 0 && m_tagPosition.getY() == 0) {
            diff_rotation = new Rotation2d();
        } else {
            Pose2d diff = current_pose.relativeTo(m_tagPosition).rotateBy(Rotation2d.kPi);
            SmartDashboard.putString("diff", diff.getX() + " " + diff.getY() + " " + diff.getRotation().getDegrees());

            double current_rotation = current_pose.getRotation().getRadians();
    
            diff_rotation = new Rotation2d(diff.getRotation().getRadians() - current_rotation);
        }
        SmartDashboard.putString("Rotation", diff_rotation.toString());
        // double tx = (LimelightHelpers.getTX("limelight") / 360) * (Math.PI * 2);

        m_drivetrain.drive(control_y, control_x, -diff_rotation.getRadians() / 5, true);
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
