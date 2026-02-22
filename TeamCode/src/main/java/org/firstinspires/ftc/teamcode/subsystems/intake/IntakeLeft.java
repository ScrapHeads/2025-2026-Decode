package org.firstinspires.ftc.teamcode.subsystems.intake;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.util.BallColor.GREEN;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.BallColor;

/**
 * The IntakeSubsystem controls the intake motor responsible for
 * collecting and feeding game elements.
 *
 * <p>Provides basic methods for setting power and stopping.
 */
public class IntakeLeft implements Subsystem  {

    private final MotorEx intakeMotorLeft;

    private BallTracker ballTracker = null;

    // Default power constants (adjust as needed)
    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = -0.4;
    public static final double REST_POWER = 0.0;

    public static final double MOTOR_TPR   = 28;   // ticks per motor rev
    public static final double GEAR_RATIO  = 1;  // motor:wheel upgear
    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    /**
     * Constructs the IntakeSubsystem.
     *
     * @param hm The HardwareMap used to retrieve the motor device.
     */
    public IntakeLeft(HardwareMap hm) {
        intakeMotorLeft = new MotorEx(hm, "intakeLeft"); // name must match configuration

        intakeMotorLeft.setInverted(false);

        intakeMotorLeft.setZeroPowerBehavior(BRAKE);

        // ensure stopped at init
        stopIntake();
    }

    /**
     * Constructs the IntakeSubsystem.
     *
     * @param hm The HardwareMap used to retrieve the motor device.
     */
    public IntakeLeft(HardwareMap hm, BallTracker ballTracker) {
        intakeMotorLeft = new MotorEx(hm, "intakeLeft"); // name must match configuration

        intakeMotorLeft.setInverted(false);

        intakeMotorLeft.setZeroPowerBehavior(BRAKE);

        this.ballTracker = ballTracker;

        // ensure stopped at init
        stopIntake();
    }

    public void setLeftPower (double power) {
        intakeMotorLeft.set(power);
    }

    /**
     * Stops the both intake motor immediately.
     */
    public void stopIntake() {
        intakeMotorLeft.stopMotor();
    }

    public void stopLeft () {intakeMotorLeft.stopMotor();}

    /**
     * @return The current power being applied to the intake motor.
     */
    public double getPowerLeft() {
        return intakeMotorLeft.get();
    }


    @Override
    public void periodic() {
        tele.addData("Intake Power", toString());

        if (ballTracker != null) {
            ballTracker.setLeftIntakePower(intakeMotorLeft.get());
        } else {
            tele.addLine("Ball Tracker null intake left");
        }

        TelemetryPacket packet = new TelemetryPacket();

        dashboard.sendTelemetryPacket(packet);
    }
}
