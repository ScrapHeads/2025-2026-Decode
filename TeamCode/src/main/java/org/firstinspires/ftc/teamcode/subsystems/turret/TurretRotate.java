package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RilLib.Control.PID.PIDController;
import org.firstinspires.ftc.teamcode.state.RobotState;

@Config
public class TurretRotate implements Subsystem {
    /**
     * Holds all tunable parameters and control state for the launcher.
     * Marked public so FTC Dashboard can reflect values in the Config tab.
     */
    public static class Params {
        public double targetPos = 0.0;

        /** Whether the PID control loop is enabled. */
        public boolean enabledPid = true;

        public double kp = 0.0002;
        public double ki = 0.00007;
        public double kd = 0.0;
    }

    public static Params PARAMS = new Params();

    public final ServoEx rotator;
    public final MotorEx encoder;

    public PIDController pid = new PIDController(PARAMS.kp, PARAMS.ki, PARAMS.kd);

    // Left Turning
    public final double MAX_ENCODER_VALUE = 15600;
    public final double MAX_ENCODER_POWER_LIMIT = MAX_ENCODER_VALUE - 821;
    // Right Turning
    public final double MIN_ENCODER_VALUE = -15600;
    public final double MIN_ENCODER_POWER_LIMIT = MIN_ENCODER_VALUE + 821;


    public static final double TICKS_PER_DEGREE = 143.36;

    public TurretRotate (HardwareMap hm) {
        rotator = new SimpleServo(hm, "turretRotate", -1, 1);
        rotator.turnToAngle(0);

        encoder = new MotorEx(hm, "feeder");
//        encoder.stopAndResetEncoder();

        //TODO Find proper Tolerance
        pid.setTolerance(TICKS_PER_DEGREE);
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
        PARAMS.targetPos = clampIfOutOfRange(MIN_ENCODER_VALUE, MAX_ENCODER_VALUE, targetPos);
    }

    public void resetEncoder () {encoder.stopAndResetEncoder();}

    public double getTurretAngle () {return encoder.getCurrentPosition() / TICKS_PER_DEGREE;}

    private double clampIfOutOfRange(double min, double max, double value) {
        return Math.max(min, Math.min(value, max));
    }

    public void enablePID () {
        PARAMS.enabledPid = true;
    }

    public void disablePID () {
        PARAMS.enabledPid = false;
    }

    public boolean isInRange () {
        return Math.abs(encoder.getCurrentPosition() - PARAMS.targetPos) < TICKS_PER_DEGREE * 2;
    }

    @Override
    public void periodic() {
//        pid.setPID(PARAMS.kp, PARAMS.ki, PARAMS.kd);

        RobotState.getInstance().setTurretAngle(getTurretAngle());
        RobotState.getInstance().setIsTurretRotating(isInRange());

        double output = 0;

        if (PARAMS.enabledPid) {
            output = -pid.calculate(encoder.getCurrentPosition(), PARAMS.targetPos);
            if (MAX_ENCODER_POWER_LIMIT >= getTurretAngle() && output > .3) {
                pid.reset();
                output = .3;
            } else if (MIN_ENCODER_POWER_LIMIT >= getTurretAngle() && output > -.3) {
                output = -.3;
                pid.reset();
            }
            turnPower(clampIfOutOfRange(-1, 1, output));
        }

        tele.addData("Rotator power", "%.1f°", getPower());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret rot Encoder", encoder.getCurrentPosition());
        packet.put("Turret location", getTurretAngle());
        packet.put("PID output", output);
        dashboard.sendTelemetryPacket(packet);
    }
}
