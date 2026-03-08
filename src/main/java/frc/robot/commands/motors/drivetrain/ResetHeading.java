package frc.robot.commands.motors.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetHeading {
  public static class ResetHeadingArbitrary extends Command {
    private final SwerveDrivetrain m_drivetrain;
    private final Double m_direction;

    public ResetHeadingArbitrary(SwerveDrivetrain drivetrain, double direction) {
      m_drivetrain = drivetrain;
      m_direction = direction;
      addRequirements(drivetrain);
    }

    @Override
    public void execute() {
      double angle = m_drivetrain.isBlueAlliance() ? m_direction : (m_direction + Math.PI) % (2 * Math.PI);
      m_drivetrain.adjustGyro(angle);
      Pose2d current_pose = m_drivetrain.getPose();
      Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
      m_drivetrain.resetPose(pose);
    }
  }

  public static class ResetHeadingForward extends ResetHeadingArbitrary {
    public ResetHeadingForward(SwerveDrivetrain drivetrain) {
      super(drivetrain, 0);
    }
  }

  public static class ResetHeadingBackward extends ResetHeadingArbitrary {
    public ResetHeadingBackward(SwerveDrivetrain drivetrain) {
      super(drivetrain, Math.PI);
    }
  }
}
