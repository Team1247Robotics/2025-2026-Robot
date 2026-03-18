
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
	private CommandJoystick controller;

	public DriveUsingAprilTagCamera(SwerveDrivetrain drivetrain, PhotonVision.PhotonVisionEstimationSubsystem camera, CommandJoystick controller) {
		this.drivetrain = drivetrain;
		this.camera = camera;
		this.controller = controller;

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
		// TODO Fix me
		/* drivetrain.drive(
			Controller.applyDriveYFilters(controller::getLeftY), // Gets the forward/backward input from the controller and apply filters.
      		Controller.applyDriveXFilters(controller::getLeftX), // Gets the left/right input from the controller and apply filters.
			-angle/30.00, // Uses the angle to the target to determine the rotation speed, with a maximum of 1 when the angle is 90 degrees.
			true); // Field-oriented control is enabled to allow the robot to drive in the direction of the target regardless of its current orientation.
		*/
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
