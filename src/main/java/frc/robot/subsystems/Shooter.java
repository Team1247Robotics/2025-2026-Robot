package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoMotorContants;

public class Shooter extends SubsystemBase {
    private static final int SHOOTER_CAN_ID = 11;
    private static final double SHOOTER_RPM = 4000;

    private static final double SHOOTER_PID_P = 0.0001;
    private static final double SHOOTER_PID_I = 0.0;
    private static final double SHOOTER_PID_D = 0.0;

    private static final double NOMINAL_VOLTAGE = 12.0; // volts
    private static final double SHOOTER_FEED_FORWARD_KV = NOMINAL_VOLTAGE / NeoMotorContants.kFreeSpeedRpm; // volts per RPM

    private final SparkMax shooterMotor;
    private final SparkClosedLoopController closedLoopController;

    public Shooter() {
        shooterMotor = new SparkMax(SHOOTER_CAN_ID, MotorType.kBrushless);
        closedLoopController = shooterMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop.pid(
                SHOOTER_PID_P,
                SHOOTER_PID_I,
                SHOOTER_PID_D
                ).feedForward
                .kV(SHOOTER_FEED_FORWARD_KV);


        config.idleMode(IdleMode.kBrake);

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /**
     * Shoot the ball out of the robot
     */
    public void shoot() {
        closedLoopController.setSetpoint(SHOOTER_RPM, ControlType.kVelocity);
    }


    /**
     * Stop the shooter motor
     */
    public void stop() {
        closedLoopController.setSetpoint(0.0, ControlType.kVelocity);
    }
}
