
package frc.robot.commands.motors.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Aborts the current turn command.
 */
public class AbortTurn extends InstantCommand {

	private SwerveDrivetrain drivetrain;

	public AbortTurn(SwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called once when this command runs
	@Override
	public void initialize() {
		System.out.println("AbortTurn: initialize");
		drivetrain.stop();
	}

}
