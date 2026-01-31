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

public class Intake extends SubsystemBase {
    private static final int INTAKE_CAN_ID = 10;
    private static final double INTAKE_RPM = 2000;
    private static final double DISPENSE_RPM = -2000;

    private static final double INTAKE_PID_P = 0.0001;
    private static final double INTAKE_PID_I = 0.0;
    private static final double INTAKE_PID_D = 0.0;

    private static final double NOMINAL_VOLTAGE = 12.0; // volts
    private static final double INTAKE_FEED_FORWARD_KV = NOMINAL_VOLTAGE / NeoMotorContants.kFreeSpeedRpm; // volts per RPM

    private final SparkMax intakeMotor;
    private final SparkClosedLoopController closedLoopController;

    public Intake() {
        intakeMotor = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushless);
        closedLoopController = intakeMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop.pid(
                INTAKE_PID_P,
                INTAKE_PID_I,
                INTAKE_PID_D).feedForward.kV(INTAKE_FEED_FORWARD_KV);

        config.idleMode(IdleMode.kBrake);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * intale
     */
    public void intake() {
        aspire();
    }

    /**
     * Pull into the robot
     */
    public void aspire() {
        closedLoopController.setSetpoint(INTAKE_RPM, ControlType.kVelocity);
    }

    /**
     * Dispense out of the robot
     */
    public void dispense() {
        closedLoopController.setSetpoint(DISPENSE_RPM, ControlType.kVelocity);
    }

    /**
     * Stop the intake motor
     */
    public void stop() {
        closedLoopController.setSetpoint(0.0, ControlType.kVelocity);
    }
}