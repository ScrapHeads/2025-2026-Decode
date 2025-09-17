package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LauncherBall implements Subsystem {

    public static class Params {
        // Target and readiness tunables
        public double targetRpm = 6000;
        public  double readyToleranceRpm = 50;
        public double readyHoldTimeSeconds = 0.10;

        // Ready to launch state
        public boolean isReadyToLaunch = false;
        public long inTolStartNanos = 0L;

        // Control state
        public boolean enabled = false;
        public double currentTargetRpm = 0.0;          // ramped setpoint
        public double maxAccelRpmPerSec = 10000.0;     // ramp limit
        public long lastLoopNanos = 0L;

        // PID (per-motor)
        public final PIDController leftPid  = new PIDController(0.3, 0.0, 0.0);
        public final PIDController rightPid = new PIDController(0.3, 0.0, 0.0);

        // Simple feedforward (at wheel rpm)
        public double kS = 0.05;
        public double kV = 1.0 / 9000.0;  // ~full power near 9000 rpm wheel speed
    }

    // Instance of params holder
    private final Params params = new Params();

    private final MotorEx leftShooter;
    private final MotorEx rightShooter;

    // Encoder resolution calculations
    public static final double MOTOR_TPR   = 28;   // ticks per motor rev
    public static final double GEAR_RATIO  = 1.5;  // motor:wheel upgear
    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    public LauncherBall(HardwareMap hm) {
        leftShooter  = new MotorEx(hm, "leftShooter");
        rightShooter = new MotorEx(hm, "rightShooter");

        leftShooter.setInverted(false);
        rightShooter.setInverted(false);

        leftShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // We control power ourselves
        leftShooter.setRunMode(Motor.RunMode.RawPower);
        rightShooter.setRunMode(Motor.RunMode.RawPower);

        // If you later use PIDController#setTolerance, use params.readyToleranceRpm
        // params.leftPid.setTolerance(params.readyToleranceRpm);
        // params.rightPid.setTolerance(params.readyToleranceRpm);
    }

    // ------------- Public API -------------
    public void enable() {
        params.enabled = true;
        params.inTolStartNanos = 0L;
        params.isReadyToLaunch = false;
        params.leftPid.reset();
        params.rightPid.reset();
        // Start a gentle ramp from current speed
        params.currentTargetRpm = getAverageRPM();
    }

    public void disable() {
        params.enabled = false;
        stop();
    }

    /** Open-loop power test (bypasses PID). */
    public void setPower(double power) {
        leftShooter.set(power);
        rightShooter.set(power);
    }

    public void stop() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
        params.inTolStartNanos = 0L;
        params.isReadyToLaunch = false;
    }

    // ----------------- Target & tuning -----------------
    public void setTargetRpm(double rpm) { params.targetRpm = Math.max(0, rpm); }
    public double getTargetRpm() { return params.targetRpm; }

    public void setReadyToleranceRpm(double tol) { params.readyToleranceRpm = Math.max(0, tol); }
    public double getReadyToleranceRpm() { return params.readyToleranceRpm; }

    public void setReadyHoldTimeSeconds(double sec) { params.readyHoldTimeSeconds = Math.max(0, sec); }
    public double getReadyHoldTimeSeconds() { return params.readyHoldTimeSeconds; }

    // ----------------- Telemetry helpers -----------------
    public double getLeftTicksPerSec()  { return leftShooter.getVelocity(); }
    public double getRightTicksPerSec() { return rightShooter.getVelocity(); }

    public double getLeftRPM()  { return (getLeftTicksPerSec()  * 60.0) / TICKS_PER_REV; }
    public double getRightRPM() { return (getRightTicksPerSec() * 60.0) / TICKS_PER_REV; }
    public double getAverageRPM() { return (getLeftRPM() + getRightRPM()) / 2.0; }

    public double rpmToTicksPerSec(double rpm) { return (rpm * TICKS_PER_REV) / 60.0; }

    public boolean isReadyToLaunch() { return params.isReadyToLaunch; }

    @Override
    public void periodic() {
        final long now = System.nanoTime();
        final double dt = (params.lastLoopNanos == 0L) ? 0.02 : (now - params.lastLoopNanos) / 1e9;
        params.lastLoopNanos = now;

        if (params.enabled) {
            // 1) Ramp target to avoid brownouts
            final double maxStep = params.maxAccelRpmPerSec * dt;
            final double delta = params.targetRpm - params.currentTargetRpm;
            if (Math.abs(delta) > maxStep) {
                params.currentTargetRpm += Math.copySign(maxStep, delta);
            } else {
                params.currentTargetRpm = params.targetRpm;
            }

            // 2) Measure
            final double leftRpm  = getLeftRPM();
            final double rightRpm = getRightRPM();

            // 3) PID (per motor) — calculate(measurement, setpoint) returns correction in "power" units
            final double leftPidOut  = params.leftPid.calculate(leftRpm,  params.currentTargetRpm);
            final double rightPidOut = params.rightPid.calculate(rightRpm, params.currentTargetRpm);

            // 4) Feedforward at the ramped setpoint (same FF to both)
            final double ff = params.kS + params.kV * params.currentTargetRpm;

            // 5) Sum and clamp
            double lOut = clamp(ff + leftPidOut,  0.0, 1.0);
            double rOut = clamp(ff + rightPidOut, 0.0, 1.0);

            // 6) Apply
            leftShooter.set(lOut);
            rightShooter.set(rOut);
        }

        // Readiness logic (unchanged)
        double leftErr  = Math.abs(getLeftRPM()  - params.targetRpm);
        double rightErr = Math.abs(getRightRPM() - params.targetRpm);
        boolean inTol   = leftErr <= params.readyToleranceRpm && rightErr <= params.readyToleranceRpm;

        if (inTol) {
            if (params.inTolStartNanos == 0L) params.inTolStartNanos = now;
            double heldSec = (now - params.inTolStartNanos) / 1e9;
            params.isReadyToLaunch = heldSec >= params.readyHoldTimeSeconds;
        } else {
            params.inTolStartNanos = 0L;
            params.isReadyToLaunch = false;
        }

        // Optional telemetry for tuning (enable MultipleTelemetry in your OpMode):
        // telemetry.addData("TgtRPM", params.targetRpm);
        // telemetry.addData("CurTgt", params.currentTargetRpm);
        // telemetry.addData("L/RPM", getLeftRPM());
        // telemetry.addData("R/RPM", getRightRPM());
        // telemetry.addData("Ready", params.isReadyToLaunch);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
