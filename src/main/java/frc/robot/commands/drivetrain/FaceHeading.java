package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Controller;

public class FaceHeading extends Command {
    protected final DriveSubsystem m_drivetrain;
    private Rotation2d m_target;
    protected DoubleSupplier m_xSupplier;
    protected DoubleSupplier m_ySupplier;
    protected boolean m_doFilters = false;
    protected boolean m_fieldRelative = true;

    private PIDController m_pid = new PIDController(0.7, 0.0, 0);

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

    public FaceHeading(DriveSubsystem drivetrain, double target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, new Rotation2d(target), xSupplier, ySupplier);
    }

    public FaceHeading(DriveSubsystem drivetrain, Rotation2d target) {
        this(drivetrain, target, () -> 0.0, () -> 0.0);
    }

    public FaceHeading(DriveSubsystem drivetrain, double target) {
        this(drivetrain, new Rotation2d(target));
    }

    protected FaceHeading(DriveSubsystem drivetrain) {
        this(drivetrain, 0);
    }

    protected FaceHeading(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, 0, xSupplier, ySupplier);
    }

    public Rotation2d updateHeading(Rotation2d target) {
        m_target = target;
        return m_target;
    }

    public double updateHeading(double target) {
        return updateHeading(new Rotation2d(target)).getRadians();
    }

    public FaceHeading applyControllerFilters(boolean set) {
        m_doFilters = set;
        return this;
    }

    public FaceHeading setFieldRelative(boolean set) {
        m_fieldRelative = set;
        return this;
    }

    protected double getInputX() {
        if (m_doFilters) {
            return Controller.applyDriveXFilters(m_xSupplier);
        } else {
            return m_xSupplier.getAsDouble();
        }
    }

    protected double getInputY() {
        if (m_doFilters) {
            return -Controller.applyDriveYFilters(m_ySupplier);
        } else {
            return m_ySupplier.getAsDouble();
        }
    }

    protected void pointToSetpoint(double x, double y, Rotation2d target) {
        double robotHeading = m_drivetrain.getPose().getRotation().getRadians();

        double direct = target.getRadians();
        double indirect = direct - Math.PI * 2;

        double directDistance = Math.abs(direct - robotHeading);
        double indirectDistance = Math.abs(indirect - robotHeading);

        double targetWraparound = directDistance < indirectDistance ? direct : indirect;

        double turnValue = m_pid.calculate(robotHeading, targetWraparound);

        m_drivetrain.drive(x, y, turnValue, m_fieldRelative);
    }

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
