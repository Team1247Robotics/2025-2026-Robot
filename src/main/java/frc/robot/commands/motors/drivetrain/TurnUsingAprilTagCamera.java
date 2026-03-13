
package frc.robot.commands.motors.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.sensors.PhotonVision;

/**
 * Command to turn the robot towards the target using the angle to the target from the AprilTag camera to assist in aiming towards the target.
 * 
 * The robot will continuously turn towards the target until it is considered to have reached the target, which is determined by the angle to the target being less than a certain threshold for a minimum number of consecutive iterations.
 */
public class TurnUsingAprilTagCamera extends Command {

	private SwerveDrivetrain drivetrain; // The drivetrain subsystem that this command will control to turn the robot towards the target.
	private PhotonVision.PhotonVisionEstimationSubsystem camera; // The camera subsystem that provides the angle to the target for this command to use in turning the robot.

	// The minimum number of consecutive iterations that the robot must be on target
	// (angle to target is less than a certain threshold) before we consider the robot to have reached the target and stop turning.
	// This is used to prevent the robot from stopping prematurely due to temporary fluctuations in the angle to the target.
	public final static int TURN_USING_CAMERA_ON_TARGET_MINIMUM_COUNT = 10;

	// The angle threshold in degrees for considering the robot to be on target when turning using the camera.
	// If the absolute value of the angle to the target is less than this threshold, we consider the robot to be on target.
	// TODO adjust as needed (probably no less than 1.0 and no more than 5.0)
	public final static double TURN_USING_CAMERA_ANGLE_THRESHOLD = 3.0;

	public int onTargetCountTurningUsingCamera; // Counter for the number of consecutive iterations that the robot has been on target while turning using the camera.


	public TurnUsingAprilTagCamera(SwerveDrivetrain drivetrain, PhotonVision.PhotonVisionEstimationSubsystem camera) {
		this.drivetrain = drivetrain;
		this.camera = camera;
		
		addRequirements(drivetrain); // Declare that this command requires the drivetrain subsystem.
	}
	
	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("TurnUsingCamera: initialize");
		onTargetCountTurningUsingCamera = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

		double angle = camera.getLatestAngleToTarget(); // Gets the angle to the target from the camera in degrees, with left being the positive direction.
		angle = MathUtil.clamp(angle, -90, 90); // Clamps the angle to the range [-90, 90] degrees to prevent excessive rotation.

		drivetrain.drive(
			0,
			0,
			-angle/30.00, // Uses the angle to the target to determine the rotation speed, with a maximum of 1 when the angle is 30 degrees.
			true);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return !tripleCheckTurnUsingCamera();
	}

	public boolean tripleCheckTurnUsingCamera()
	{
		boolean isTurningUsingCamera = true;
		
		{
			double angle = camera.getLatestAngleToTarget(); // Gets the angle to the target from the camera in degrees, with left being the positive direction.

			boolean isOnTarget = Math.abs(angle) < TURN_USING_CAMERA_ANGLE_THRESHOLD; // angle in degree

			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCountTurningUsingCamera++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCountTurningUsingCamera > 0) { // even though we were on target at least once during a previous iteration
					onTargetCountTurningUsingCamera = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (turning using camera).");
				} else {
					// we are definitely turning
				}
			}
			
			if (onTargetCountTurningUsingCamera > TURN_USING_CAMERA_ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
				isTurningUsingCamera = false;
			}
			
			if (!isTurningUsingCamera) {
				System.out.println("You have reached the target (turning using camera).");
				//stop();				 
			}
		}

		return isTurningUsingCamera;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("TurnUsingCamera: end");
		drivetrain.stop();
	}
}
