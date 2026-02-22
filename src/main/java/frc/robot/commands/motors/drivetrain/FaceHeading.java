package frc.robot.commands.motors.drivetrain;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GyroConstants;
import frc.robot.commands.generics.GenericAwaitBaseTargetWithinError;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Controller;
import frc.robot.utils.Targeting;

public interface FaceHeading {
  interface Await {
    class Actively extends GenericAwaitBaseTargetWithinError {
      protected final DriveSubsystem m_drivetrain;
  
      /**
       * Offset in radians to face from the target. This is applied immeditely before calculating velocities and cannot be avoided.
       */
      protected Rotation2d m_offset = Rotation2d.kZero;
  
      /**
       * Target heading to reach.
       * @implNote
       * Does not need to be set when extending.
       */
      protected Supplier<Rotation2d> m_headingTargetSupplier;
  
      /**
       * Function to get target x velocity or effort.
       */
      protected DoubleSupplier m_xSupplier = () -> 0;
      /**
       * Function to get target y velocity or effort.
       */
      protected DoubleSupplier m_ySupplier = () -> 0;
  
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
       * @param target - Rotation target heading supplier.
       * @param xSupplier - Function that when called returns the target x velocity or effort.
       * @param ySupplier - Function that when called returns the target y velocity or effort.
       */
      public Actively(
          DriveSubsystem drivetrain,
          Supplier<Rotation2d> target,
          DoubleSupplier xSupplier,
          DoubleSupplier ySupplier
      ) {
        super(() -> Targeting.getAngularOffset(drivetrain.getPose().getRotation(), target.get()), 0, GyroConstants.TargetAngleAllowableError.abs(Radians));
        this.m_drivetrain = drivetrain;
        this.m_headingTargetSupplier = target;
        this.m_xSupplier = xSupplier;
        this.m_ySupplier = ySupplier;
        m_pid.enableContinuousInput(0, Math.PI * 2);
        setUseRadianWraparound(true);
      }
  
      /**
       * @param drivetrain - The drivetrain.
       * @param target - Rotation target heading.
       * @param xSupplier - Function that when called returns the target x velocity or effort.
       * @param ySupplier - Function that when called returns the target y velocity or effort.
       */
      public Actively(
          DriveSubsystem drivetrain,
          Rotation2d target,
          DoubleSupplier xSupplier,
          DoubleSupplier ySupplier
      ) {
        this(drivetrain, () -> target, xSupplier, ySupplier);
      }
  
      /**
       * @param drivetrain - The drivetrain.
       * @param target - Double representing the target heading in radians.
       * @param xSupplier - Function that when called returns the target x velocity or effort.
       * @param ySupplier - Function that when called returns the target y velocity or effort.
       */
      public Actively(DriveSubsystem drivetrain, double target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, new Rotation2d(target), xSupplier, ySupplier);
      }
  
      /**
       * Constructor to stand still while facing a target.
       * @param drivetrain - The drivetrain.
       * @param target - Target heading.
       */
      public Actively(DriveSubsystem drivetrain, Rotation2d target) {
        this(drivetrain, target, () -> 0.0, () -> 0.0);
      }
  
      /**
       * Constructor to stand still while facing a target.
       * @param drivetrain - The drivetrain.
       * @param target - Double representing the target heading in radians.
       */
      public Actively(DriveSubsystem drivetrain, double target) {
        this(drivetrain, new Rotation2d(target));
      }
  
      /**
       * Protected constructor that does not require anything except a drivetrain.
       * Only call this if you intend to use {@link #pointToSetpoint(double, double, Rotation2d)} or intend to stand still and use {@link #pointToSetpoint(Rotation2d)}.
       */
      protected Actively(DriveSubsystem drivetrain) {
        this(drivetrain, 0);
      }
  
      /**
       * Protected constructor that does not require a target.
       * Only call this if you intend to use {@link #pointToSetpoint(Rotation2d)}.
       * @param drivetrain - The drivetrain.
       * @param xSupplier - Function that when called returns the target x velocity or effort.
       * @param ySupplier - Function that when called returns the target y velocity or effort.
       */
      protected Actively(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(drivetrain, 0, xSupplier, ySupplier);
      }
  
      /**
       * Updates the target heading.
       * @param target - New target heading supplier.
       * @return The new target.
       */
      public void updateHeading(Supplier<Rotation2d> target) {
        m_headingTargetSupplier = target;
        this.m_base = () -> Targeting.getAngularOffset(m_drivetrain.getHeading(), m_headingTargetSupplier.get());
      }

      /**
       * Updates the target heading.
       * @param target - New target heading.
       * @return The new target.
       */
      public void updateHeading(Rotation2d target) {
        updateHeading(() -> target);
      }
  
      /**
       * Updates the target heading.
       * @param target - Double reprenting the target heading in radians.
       * @return The new angle in radians.
       */
      public void updateHeading(double target) {
        updateHeading(Rotation2d.fromRadians(target));
      }
  
      /**
       * Set {@link #m_doFilters}. Can be chained with constructor.
       * 
       * Filters apply deadband and the necessary inversions to pass the controller value accessors as-is.
       * @param set - Value to set it to.
       * @return self
       */
      public Actively applyControllerFilters(boolean set) {
        m_doFilters = set;
        return this;
      }
  
      /**
       * Set {@link #m_fieldRelative}. Can be chained with constructor.
       * @param set
       * @return self
       */
      public Actively setFieldRelative(boolean set) {
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
  
        double direct = ((target.plus(m_offset).getRadians()) % (Math.PI * 2));
        // SmartDashboard.putNumber("Heading Target Offset", -(direct - robotHeading - m_offset.getRadians()));
        SmartDashboard.putNumber("Heading Target Offset", Targeting.getAngularOffset(robotHeading, direct));
  
        return m_pid.calculate(robotHeading - m_offset.getRadians(), target.getRadians());
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
        pointToSetpoint(m_headingTargetSupplier.get());
      }
    }
  
    class Passively extends Actively {
  
      /**
       * Constructor to stand still while facing a target.
       * @param drivetrain - The drivetrain.
       * @param target - Target heading.
       */
      public Passively(DriveSubsystem drivetrain, Rotation2d target) {
        super(drivetrain, target);
      }
  
      /**
       * Constructor to stand still while facing a target.
       * @param drivetrain - The drivetrain.
       * @param target - Double representing the target heading in radians.
       */
      public Passively(DriveSubsystem drivetrain, double target) {
        super(drivetrain, target);
      }
  
      /**
       * Protected constructor that does not require anything except a drivetrain.
       * Only call this if you intend to use {@link #pointToSetpoint(double, double, Rotation2d)} or intend to stand still and use {@link #pointToSetpoint(Rotation2d)}.
       */
      protected Passively(DriveSubsystem drivetrain) {
        super(drivetrain);
      }

      @Override
      public void execute() {
        // do nothing
      }
    }
  }

  class Indefinitely extends Await.Actively {
    public Indefinitely(
        DriveSubsystem drivetrain,
        Rotation2d target,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {
      super(drivetrain, target, xSupplier, ySupplier);
    }

    /**
     * @param drivetrain - The drivetrain.
     * @param target - Double representing the target heading in radians.
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    public Indefinitely(DriveSubsystem drivetrain, double target, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      super(drivetrain, target, xSupplier, ySupplier);
    }

    /**
     * Constructor to stand still while facing a target.
     * @param drivetrain - The drivetrain.
     * @param target - Target heading.
     */
    public Indefinitely(DriveSubsystem drivetrain, Rotation2d target) {
      super(drivetrain, target);
    }

    /**
     * Constructor to stand still while facing a target.
     * @param drivetrain - The drivetrain.
     * @param target - Double representing the target heading in radians.
     */
    public Indefinitely(DriveSubsystem drivetrain, double target) {
      super(drivetrain, target);
    }

    /**
     * Protected constructor that does not require anything except a drivetrain.
     * Only call this if you intend to use {@link #pointToSetpoint(double, double, Rotation2d)} or intend to stand still and use {@link #pointToSetpoint(Rotation2d)}.
     */
    protected Indefinitely(DriveSubsystem drivetrain) {
      super(drivetrain);
    }
  
    /**
     * Protected constructor that does not require a target.
     * Only call this if you intend to use {@link #pointToSetpoint(Rotation2d)}.
     * @param drivetrain - The drivetrain.
     * @param xSupplier - Function that when called returns the target x velocity or effort.
     * @param ySupplier - Function that when called returns the target y velocity or effort.
     */
    protected Indefinitely(DriveSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      super(drivetrain, xSupplier, ySupplier);
    }
  }
}
