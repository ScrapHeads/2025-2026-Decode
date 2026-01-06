package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RilLib.Control.PID.PIDController;

public class TurretRotate implements Subsystem {
    /**
     * Holds all tunable parameters and control state for the launcher.
     * Marked public so FTC Dashboard can reflect values in the Config tab.
     */
    public static class Params {
        public double targetPos = 0.0;

        /** Whether the PID control loop is enabled. */
        public boolean enabledPid = false;
    }

    public static Params PARAMS = new Params();

    public final ServoEx rotator;
    public final MotorEx encoder;

    public PIDController pid = new PIDController(0,0,0);

    //TODO find the correct range
    public final double MAX_ENCODER_VALUE = 0;
    public final double MIN_ENCODER_VALUE = 0;

    public static final double TICKS_PER_DEGREE = 0;

    public TurretRotate (HardwareMap hm) {
        rotator = new SimpleServo(hm, "turretRotate", -1, 1);
        rotator.turnToAngle(0);

        encoder = new MotorEx(hm, "feeder");

        //TODO Find proper Tolerance
        pid.setTolerance(0);
        pid.setIntegratorRange(-1, 1);
        pid.reset();
    }

    /**
     * Turns the servo a direction based on a -1 to 1 scale
     * @param power -1 to 1
     * */
    public void turnPower (double power) {
        rotator.turnToAngle(power);
    }

    public double getPower () {return rotator.getAngle();}

    public void setTargetPos (double targetPos) {
        double safeAngle = targetPos; // Math.max(MIN_ENCODER_VALUE, Math.min(targetPos, MAX_ENCODER_VALUE));
        PARAMS.targetPos = safeAngle;
    }

    public void resetEncoder () {encoder.stopAndResetEncoder();}


    @Override
    public void periodic() {

        if (PARAMS.enabledPid) {
            double output = pid.calculate(encoder.get(), PARAMS.targetPos);
            turnPower(output);
        }


        tele.addData("Rotator power", "%.1f°", getPower());
    }
}
