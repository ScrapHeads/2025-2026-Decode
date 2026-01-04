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
public class Intake implements Subsystem  {

    private final MotorEx intakeMotorLeft;
    private final MotorEx intakeMotorRight;

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
    public Intake(HardwareMap hm) {
        intakeMotorLeft = new MotorEx(hm, "intakeLeft"); // name must match configuration
        intakeMotorRight = new MotorEx(hm, "intakeRight");

        intakeMotorRight.setInverted(true);

        intakeMotorLeft.setZeroPowerBehavior(BRAKE);
        intakeMotorRight.setZeroPowerBehavior(BRAKE);

        // ensure stopped at init
        stopBoth();
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
    public void stopBoth() {
        intakeMotorLeft.stopMotor();
        intakeMotorRight.stopMotor();
    }

    public void stopLeft () {intakeMotorLeft.stopMotor();}
    public void stopRight () {intakeMotorRight.stopMotor();}

    /**
     * @return The current power being applied to the intake motor.
     */
    public double getPowerLeft() {
        return intakeMotorLeft.get();
    }
    public double getPowerRight() {return intakeMotorRight.get();}

    public double getTicksPerSec()  { return intakeMotorLeft.encoder.getRawVelocity(); }
    public double getShooterRPM()  { return (getTicksPerSec() * 60.0) / TICKS_PER_REV; }

    @Override
    public void periodic() {
        tele.addData("Intake Power", toString());
        tele.update();
    }

    @Override
    public String toString () {
        return "Left: " + intakeMotorLeft.get() + ", Right: " + getShooterRPM();
    }

}
