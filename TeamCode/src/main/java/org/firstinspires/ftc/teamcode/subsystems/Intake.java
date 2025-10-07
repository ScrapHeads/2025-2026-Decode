package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The IntakeSubsystem controls the intake motor responsible for
 * collecting and feeding game elements.
 *
 * <p>Provides basic methods for setting power and stopping.
 */
public class Intake implements Subsystem {

    private final MotorEx intakeMotor;

    // Default power constants (adjust as needed)
    public static final double INTAKE_POWER = -1.0;
    public static final double OUTTAKE_POWER = 1.0;

    /**
     * Constructs the IntakeSubsystem.
     *
     * @param hm The HardwareMap used to retrieve the motor device.
     */
    public Intake(HardwareMap hm) {
        intakeMotor = new MotorEx(hm, "intake"); // name must match configuration
        intakeMotor.setZeroPowerBehavior(BRAKE);
        intakeMotor.stopMotor(); // ensure stopped at init
    }

    /**
     * Sets the motor power.
     *
     * @param power Power value (-1.0 to 1.0)
     */
    public void setPower(double power) {
        intakeMotor.set(power);
    }

    /**
     * Stops the intake motor immediately.
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * @return The current power being applied to the intake motor.
     */
    public double getPower() {
        return intakeMotor.get();
    }

    @Override
    public void periodic() {
        tele.addData("Intake Power", "%.2f", getPower());
    }
}
