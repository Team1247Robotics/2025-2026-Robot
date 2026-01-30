package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Controller;

/**
 * Indefinitely face a specified heading
 * 
 * @implNote
 * Use {@link #pointToSetpoint} to specify a heading to face dynamically.
 * @implNote
 * Protected constructor {@link #FaceHeading(DriveSubsystem)} or {@link #FaceHeading(DriveSubsystem, DoubleSupplier, DoubleSupplier)} can be paired with {@link #pointToSetpoint} to instantiate without supplying a target.
 * @implNote
 * Calling {@link #pointToSetpoint(Rotation2d)} will automatically use X and Y speed suppliers.
 */
public class FaceHeading extends Command {
    protected final DriveSubsystem m_drivetrain;

    /**
     * Target heading to reach.
     * @implNote
     * Does not need to be set when extending.
     */
    private Rotation2d m_target;

    /**
     * Function to get target x velocity or effort.
     */
    protected DoubleSupplier m_xSupplier;
    /**
     * Function to get target y velocity or effort.
     */
    protected DoubleSupplier m_ySupplier;

    /**
     * Applies controller specific filters such as deadband.
     */
    protected boolean m_doFilters = false;

    /**
     * If driving should use field relative movement.
     */
    protected boolean m_fieldRelative = true;

    
    private PIDController m_pid = new PIDController(0.7, 0.0, 0);

    /**
     * @param drivetrain - The drivetrain.
     * @param target - Rotation target heading.
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    public FaceHeading(
        DriveSubsystem drivetrain,
        Rotation2d target,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {
        this.m_drivetrain = drivetrain;
        this.m_target = target;
        this.m_xSupplier = xSupplier;
        this.m_ySupplier = ySupplier;
    }

    /**
     * @param drivetrain - The drivetrain.
     * @param target - Double representing the target heading in radians.
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    public FaceHeading(DriveSubsystem drivetrain, double target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, new Rotation2d(target), xSupplier, ySupplier);
    }

    /**
     * Constructor to stand still while facing a target.
     * @param drivetrain - The drivetrain.
     * @param target - Target heading.
     */
    public FaceHeading(DriveSubsystem drivetrain, Rotation2d target) {
        this(drivetrain, target, () -> 0.0, () -> 0.0);
    }

    /**
     * Constructor to stand still while facing a target.
     * @param drivetrain - The drivetrain.
     * @param target - Double representing the target heading in radians.
     */
    public FaceHeading(DriveSubsystem drivetrain, double target) {
        this(drivetrain, new Rotation2d(target));
    }

    /**
     * Protected constructor that does not require anything except a drivetrain.
     * Only call this if you intend to use {@link #pointToSetpoint(double, double, Rotation2d)} or intend to stand still and use {@link #pointToSetpoint(Rotation2d)}.
     */
    protected FaceHeading(DriveSubsystem drivetrain) {
        this(drivetrain, 0);
    }

    /**
     * Protected constructor that does not require a target.
     * Only call this if you intend to use {@link #pointToSetpoint(Rotation2d)}.
     * @param drivetrain - The drivetrain.
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    protected FaceHeading(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, 0, xSupplier, ySupplier);
    }

    /**
     * Updates the target heading.
     * @param target - New target heading.
     * @return The new target.
     */
    public Rotation2d updateHeading(Rotation2d target) {
        m_target = target;
        return m_target;
    }

    /**
     * Updates the target heading.
     * @param target - Double reprenting the target heading in radians.
     * @return The new angle in radians.
     */
    public double updateHeading(double target) {
        return updateHeading(new Rotation2d(target)).getRadians();
    }

    /**
     * Set {@link #m_doFilters}. Can be chained with constructor.
     * @param set - Value to set it to.
     * @return self
     */
    public FaceHeading applyControllerFilters(boolean set) {
        m_doFilters = set;
        return this;
    }

    /**
     * Set {@link #m_fieldRelative}. Can be chained with constructor.
     * @param set
     * @return self
     */
    public FaceHeading setFieldRelative(boolean set) {
        m_fieldRelative = set;
        return this;
    }

    /**
     * Get input from {@link #m_xSupplier}. Applies controller filters if enabled in {@link #m_doFilters}.
     * @return Double representing target velocity
     */
    protected double getInputX() {
        if (m_doFilters) {
            return Controller.applyDriveXFilters(m_xSupplier);
        } else {
            return m_xSupplier.getAsDouble();
        }
    }

    /**
     * Get input from {@link #m_ySupplier}. Applies controller filters if enabled in {@link #m_doFilters}.
     * @return Double representing target velocity
     */
    protected double getInputY() {
        if (m_doFilters) {
            return -Controller.applyDriveYFilters(m_ySupplier);
        } else {
            return m_ySupplier.getAsDouble();
        }
    }

    /**
     * Calculate new turn value given a heading target.
     * 
     * @implNote
     * This is implicitely called by {@link #pointToSetpoint}. Visibility is protected to allow for potential future advanced usage.
     * @param target - Heading target
     * @return Double representing the velocity the PID controllers is attempting to reach.
     */
    protected double calculateTurn(Rotation2d target) {
        double robotHeading = (m_drivetrain.getPose().getRotation().getRadians() % (Math.PI * 2));

        double direct = (target.getRadians() % (Math.PI * 2));
        double indirect = direct - Math.PI * 2;

        double directDistance = Math.abs(direct - robotHeading);
        double indirectDistance = Math.abs(indirect - robotHeading);

        double targetWraparound = directDistance < indirectDistance ? direct : indirect;

        return m_pid.calculate(robotHeading, targetWraparound);
    }

    /**
     * Drive robot with X and Y velocities to face target. Must be called every tick.
     * @param x - X velocity
     * @param y - Y velocity
     * @param target - Target rotation
     */
    protected void pointToSetpoint(double x, double y, Rotation2d target) {
        double turnValue = calculateTurn(target);

        m_drivetrain.drive(x, y, turnValue, m_fieldRelative);
    }

    /**
     * Drive robot to face target. X and Y velocities pulled from {@link #m_xSupplier} and {@link #m_ySupplier}. Must be called every tick.
     * @param target - Target rotation
     */
    protected void pointToSetpoint(Rotation2d target) {
        double xInput = getInputX();
        double yInput = getInputY();

        pointToSetpoint(xInput, yInput, target);
    }

    @Override
    public void execute() {
        pointToSetpoint(m_target);
    }
}
