
package frc.robot.commands.motors.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.Controller;

/**
 * Command to drive the robot using the angle to the target from the AprilTag camera to assist in aiming towards the target.
 * 
 * The robot will continuously aim at the target while allowing the driver to control the forward/backward and left/right movement with the controller.
 * 
 * The rotation will be automatically adjusted based on the angle to the target, with a maximum rotation speed when the angle is 90 degrees.
 */
public class DriveUsingAprilTagCamera extends Command {

	private SwerveDrivetrain drivetrain;
	private PhotonVision.PhotonVisionEstimationSubsystem camera;
	private final CommandXboxController xboxController;
	private final CommandJoystick joystickController;
	private final boolean useXbox;

	public DriveUsingAprilTagCamera(SwerveDrivetrain drivetrain, PhotonVision.PhotonVisionEstimationSubsystem camera, CommandXboxController controller) {
		this.drivetrain = drivetrain;
		this.camera = camera;
		this.xboxController = controller;
		this.joystickController = null;
		this.useXbox = true;
		addRequirements(drivetrain);
	}

	public DriveUsingAprilTagCamera(SwerveDrivetrain drivetrain, PhotonVision.PhotonVisionEstimationSubsystem camera, CommandJoystick controller) {
		this.drivetrain = drivetrain;
		this.camera = camera;
		this.xboxController = null;
		this.joystickController = controller;
		this.useXbox = false;

		addRequirements(drivetrain);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("DriveUsingAprilTagCamera: initialize");
	}


	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

		double angle = camera.getLatestAngleToTarget(); // Gets the angle to the target from the camera in degrees, with left being the positive direction.
		angle = MathUtil.clamp(angle, -90, 90); // Clamps the angle to the range [-90, 90] degrees to prevent excessive rotation.

		double forward;
		double strafe;

		if (useXbox) {
			forward = Controller.applyDriveYFilters(xboxController::getLeftY);
			strafe = Controller.applyDriveXFilters(xboxController::getLeftX);
		} else {
			forward = Controller.applyDriveYFilters(joystickController::getY);
			strafe = Controller.applyDriveXFilters(joystickController::getX);
		}

		drivetrain.drive(
				forward,
				strafe,
				-angle / 30.0,
				true);
	}


	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("DriveUsingAprilTagCamera: end");
		drivetrain.stop();
	}
}
