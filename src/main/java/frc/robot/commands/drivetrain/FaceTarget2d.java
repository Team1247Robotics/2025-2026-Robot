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
    /**
     * Supplier to get the latest Pose2d every tick.
     */
    private Supplier<Pose2d> m_targetSupplier;

    /**
     * @param drivetrain - The drivetrain
     * @param targetSupplier - Function that returns a target Pose2d.
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    public FaceTarget2d(DriveSubsystem drivetrain, Supplier<Pose2d> targetSupplier, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(drivetrain, xSupplier, ySupplier);
        m_targetSupplier = targetSupplier;
    }

    /**
     * @param drivetrain - The drivetrain
     * @param target - Target Pose2d
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    public FaceTarget2d(DriveSubsystem drivetrain, Pose2d target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, () -> target, xSupplier, ySupplier);
    }

    /**
     * Protected constructor intended to be used when manually calling {@link #pointToTarget}.
     * @param drivetrain - The drivetrain
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    protected FaceTarget2d(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, () -> Pose2d.kZero, xSupplier, ySupplier);
    }

    /**
     * Protected constructor intended to be used when manually calling {@link #pointToTarget(double, double, Pose2d)}
     * @param drivetrain - The drivetrain
     */
    protected FaceTarget2d(DriveSubsystem drivetrain) {
        this(drivetrain, () -> Pose2d.kZero, () -> 0.0, () -> 0.0);
    }

    /**
     * Updates the target pose of the command.
     * @param newTarget - The new Pose2d.
     */
    public void updateTarget(Pose2d newTarget) {
        m_targetSupplier = () -> newTarget;
    }

    /**
     * Updates the function that is called when updating the target pose.
     * @param newTargetSupplier - The new Pose2d supplier.
     */
    public void updateTargetSupplier(Supplier<Pose2d>  newTargetSupplier) {
        m_targetSupplier = newTargetSupplier;
    }

    /**
     * Point to target with X and Y velocities.
     * @param inputX - Target X velocity.
     * @param inputY - Target Y velocity.
     * @param target - Target pose to face in field space.
     */
    protected void pointToTarget(double inputX, double inputY, Pose2d target) {
        Rotation2d angle = Targeting.convertFieldRelativeToRobotRelativeTranslation(m_drivetrain.getPose(), target, Translation2d.kZero).getAngle();

        pointToSetpoint(inputX, inputY, angle);
    }

    /**
     * Point to target
     * @param target - Target pose to face in field space.
     */
    protected void pointToTarget(Pose2d target) {
        Rotation2d angle = Targeting.convertFieldRelativeToRobotRelativeTranslation(m_drivetrain.getPose(), target, Translation2d.kZero).getAngle();

        pointToSetpoint(angle);
    }

    @Override
    public void execute() {
        pointToTarget(m_targetSupplier.get());
    }
}
