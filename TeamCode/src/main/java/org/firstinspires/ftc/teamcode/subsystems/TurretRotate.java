package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
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
        public boolean enabledPid = false;

        public double kp = 0.0;
        public double ki = 0.0;
        public double kd = 0.0;
    }

    public static Params PARAMS = new Params();

    public final ServoEx rotator;
    public final MotorEx encoder;

    public PIDController pid = new PIDController(PARAMS.kp, PARAMS.ki, PARAMS.kd);

    // Left Turning
    public final double MAX_ENCODER_VALUE = 15600;
    // Right Turning
    public final double MIN_ENCODER_VALUE = -15600;

    public static final double TICKS_PER_DEGREE = 143.36;

    public TurretRotate (HardwareMap hm) {
        rotator = new SimpleServo(hm, "turretRotate", -1, 1);
        rotator.turnToAngle(0);

        encoder = new MotorEx(hm, "feeder");

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
        double safePos = Math.max(MIN_ENCODER_VALUE, Math.min(targetPos, MAX_ENCODER_VALUE));
        PARAMS.targetPos = safePos;
    }

    public void resetEncoder () {encoder.stopAndResetEncoder();}

    public double getTurretAngle () {return encoder.getCurrentPosition() / TICKS_PER_DEGREE;}

    @Override
    public void periodic() {
        pid.setPID(PARAMS.kp, PARAMS.ki, PARAMS.kd);

        RobotState.getInstance().setTurretAngle(getTurretAngle());

        if (PARAMS.enabledPid) {
            double output = -pid.calculate(encoder.getCurrentPosition(), PARAMS.targetPos);
            turnPower(output);
        }

        tele.addData("Rotator power", "%.1f°", getPower());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret rot Encoder", encoder.getCurrentPosition());
        packet.put("Turret location", getTurretAngle());
        dashboard.sendTelemetryPacket(packet);
    }
}
