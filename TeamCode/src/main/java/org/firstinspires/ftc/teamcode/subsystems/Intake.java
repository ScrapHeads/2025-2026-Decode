package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

/**
 * The IntakeSubsystem controls the intake motor responsible for
 * collecting and feeding game elements.
 *
 * <p>Provides basic methods for setting power and stopping.
 */
public class Intake implements Subsystem  {

    private final MotorEx intakeMotorLeft;
    private final MotorEx intakeMotorRight;

    // Default power constants (adjust as needed)
    public static final double INTAKE_POWER = -1.0;
    public static final double OUTTAKE_POWER = 1.0;
    public static final double REST_POWER = 0.0;

    /**
     * Constructs the IntakeSubsystem.
     *
     * @param hm The HardwareMap used to retrieve the motor device.
     */
    public Intake(HardwareMap hm) {
        intakeMotorLeft = new MotorEx(hm, "intakeL"); // name must match configuration
        intakeMotorRight = new MotorEx(hm, "intakeR");

        intakeMotorLeft.setZeroPowerBehavior(BRAKE);
        intakeMotorRight.setZeroPowerBehavior(BRAKE);

        // ensure stopped at init
        stop();
    }

    /**
     * Sets both of the motor power.
     *
     * @param power Power value (-1.0 to 1.0)
     */
    public void setBothPower(double power) {
        intakeMotorLeft.set(power);
        intakeMotorRight.set(power);
    }

    public void setLeftPower (double power) {
        intakeMotorLeft.set(power);
    }
    public void setRightPower (double power) { intakeMotorRight.set(power); }

    /**
     * Stops the both intake motor immediately.
     */
    public void stop() {
        intakeMotorLeft.stopMotor();
        intakeMotorRight.stopMotor();
    }

    /**
     * @return The current power being applied to the intake motor.
     */
    public double getPower() {
        return intakeMotorLeft.get();
    }

    @Override
    public void periodic() {
        tele.addData("Intake Power", toString());
    }

    @Override
    public String toString () {
        return "Left: " + intakeMotorLeft.get() + ", Right: " + intakeMotorRight.get();
    }

}
