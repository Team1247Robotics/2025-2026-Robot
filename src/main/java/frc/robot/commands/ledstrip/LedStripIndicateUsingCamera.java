
package frc.robot.commands.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.PhotonVision;
import frc.robot.subsystems.LedStrip;

/**
 * This command uses the camera to determine how to set the LED strip.
 * 
 * If the camera sees a target, it will set the LED strip to green if the target is within 5 degrees of the center,
 * yellow if it's within 15 degrees, and red if it's further than 15 degrees.
 * If the camera does not see a target, it will set the LED strip to blue.
 * This command is intended to be used while the robot is trying to aim at the target, so that the driver can get feedback
 * on how close they are to being aimed at the target.
 * It could also be used in auton to provide feedback on whether the robot is aimed at the target or not.
 */
public class LedStripIndicateUsingCamera extends Command {

	private LedStrip indicator;
	private PhotonVision.PhotonVisionEstimationSubsystem camera;

	public LedStripIndicateUsingCamera(LedStrip indicator, PhotonVision.PhotonVisionEstimationSubsystem camera) {
		this.indicator = indicator;
		this.camera = camera;

		addRequirements(indicator);
	}

	// This instant command can run disabled
	/*@Override
	public boolean runsWhenDisabled() {
		return true;
	}*/

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("LedStripIndicateUsingCamera: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		double angle = camera.getLatestAngleToTarget();  // Gets the angle to the target from the camera in degrees, with left being the positive direction.
		
		if (angle != 0.0) { // if we saw something (0.0 is the default value when there is no target, so if it's not 0.0 then we know we saw something)
			if (Math.abs(angle) < 5) { // displays green if in target
				indicator.applySolid(Color.kGreen);
			}
			else if (Math.abs(angle) < 15) { // displays yellow if close to target
				indicator.applySolid(Color.kYellow);
			}
			else { // displays red if far from target 
				indicator.applySolid(Color.kRed);
			}
		} else { // no target, so arbitrarily displays blue 
			indicator.applySolid(Color.kBlue);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false; // we are never finished
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("LedStripIndicateUsingCamera: end");
		//indicator.stop();
	}
}
