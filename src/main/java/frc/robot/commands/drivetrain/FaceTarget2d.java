package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Targeting;

/**
 * Face a field relative Pose2d.
 * 
 * @implNote
 * Subclasses can call {@link #pointToTarget} to dynamically change target more efficiently.
 * @implNote
 * Protected constructor {@link #FaceTarget2d(DriveSubsystem)} can be paired with {@link #pointToTarget} to instantiate without supplying a target.
 */
public class FaceTarget2d extends FaceHeading {
    private Supplier<Pose2d> m_targetSupplier;

    public FaceTarget2d(DriveSubsystem drivetrain, Supplier<Pose2d> targetSupplier, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(drivetrain, xSupplier, ySupplier);
        m_targetSupplier = targetSupplier;
    }

    public FaceTarget2d(DriveSubsystem drivetrain, Pose2d target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, () -> target, xSupplier, ySupplier);
    }

    protected FaceTarget2d(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, () -> Pose2d.kZero, xSupplier, ySupplier);
    }

    protected FaceTarget2d(DriveSubsystem drivetrain) {
        this(drivetrain, () -> Pose2d.kZero, () -> 0.0, () -> 0.0);
    }

    public void updateTarget(Pose2d newTarget) {
        m_targetSupplier = () -> newTarget;
    }

    public void updateTargetSupplier(Supplier<Pose2d>  newTargetSupplier) {
        m_targetSupplier = newTargetSupplier;
    }

    protected void pointToTarget(double inputX, double inputY, Pose2d target) {
        Rotation2d angle = Targeting.convertFieldRelativeToRobotRelativeTranslation(m_drivetrain.getPose(), target, Translation2d.kZero).getAngle();

        pointToSetpoint(inputX, inputY, angle);
    }

    protected void pointToTarget(Pose2d target) {
        Rotation2d angle = Targeting.convertFieldRelativeToRobotRelativeTranslation(m_drivetrain.getPose(), target, Translation2d.kZero).getAngle();

        pointToSetpoint(angle);
    }

    @Override
    public void execute() {
        pointToTarget(m_targetSupplier.get());
    }
}
