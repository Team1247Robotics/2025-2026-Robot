package frc.robot.commands.motors.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class FacePointTest extends FaceTarget2d {
    private Pose2d m_tagPosition = Pose2d.kZero;

    private Field2d m_tagOnField = new Field2d();

    /**
     * 
     * @param drivetrain
     * @param controller
     */
    public FacePointTest(DriveSubsystem drivetrain, XboxController controller) {
        super(drivetrain, controller::getLeftX, controller::getLeftY);
        this.m_fieldRelative = true;
        this.m_doFilters = true;

        m_tagOnField.setRobotPose(m_tagPosition);
    }

    @Override
    public void execute() {
        m_tagPosition = new Pose2d(12, 7.5, Rotation2d.kZero);

        m_tagOnField.setRobotPose(m_tagPosition);
        SmartDashboard.putData("Target", m_tagOnField);

        pointToTarget(m_tagPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
