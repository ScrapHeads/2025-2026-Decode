package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.blueTagPose;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.LauncherTable;
import static org.firstinspires.ftc.teamcode.Constants.redTagPose;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.InterpolatingDoubleTreeMap;
import org.firstinspires.ftc.teamcode.state.RobotState;

/**
 * Subsystem representing a single-motor flywheel launcher.
 * <p>
 * This class uses a PID + feedforward loop to hold the flywheel
 * at a desired RPM, with a configurable acceleration ramp to
 * prevent brownouts. FTC Dashboard can be used to live-tune
 * parameters via the {@link Params} inner class.
 */
@Config
public final class Launcher implements Subsystem {

    /**
     * Holds all tunable parameters and control state for the launcher.
     * Marked public so FTC Dashboard can reflect values in the Config tab.
     */
    public static class Params {
        /** Desired target wheel speed in RPM. */
        public double targetRpm = 0;

        /** Allowed RPM error margin to consider launcher "ready". */
        public double readyToleranceRpm = 200;

        /** How long (seconds) the RPM must stay in tolerance to be "ready". */
        public double readyHoldTimeSeconds = 0.10;

        // --- Runtime state ---
        /** True if within tolerance for the required hold time. */
        public boolean isReadyToLaunch = false;
        /** Timestamp when we first entered tolerance, in ns. */
        public long inTolStartNanos = 0L;
        /** Whether the PID control loop is enabled. */
        public boolean enabledPid = false;
        /** Ramped setpoint RPM used to avoid sudden current draw. */
        public double currentTargetRpm = 0.0;
        /** Max ramp rate in RPM/sec. */
        public double maxAccelRpmPerSec = 3000;
        /** Time of last loop iteration, in ns. */
        public long lastLoopNanos = 0L;

        // --- Control ---
        /** Single PID controller for the shooter motor. */
        public double PIDKp = 0.00024;
        public double PIDKi = 0.0;
        public double PIDKd = 0.0;
        public double PIDKf = 0.000175;

        /** Static feedforward to overcome friction. */
        public double feedForwardKS = 0.05;
        /** Velocity feedforward (power per RPM). */
        public double feedForwardKV = 1.0 / 6000.0;  // ~full power near 6000 RPM
    }

    /** Instance of params for this launcher. */
    public static Params PARAMS = new Params();

    public PIDController shooterPid = new PIDController(PARAMS.PIDKp, PARAMS.PIDKi, PARAMS.PIDKd);

    /** Motor driving the shooter flywheel. */
    private final MotorEx launcherLeft;
    private final MotorEx launcherRight;

    private InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

    // Encoder resolution calculations
    public static final double MOTOR_TPR   = 28;   // ticks per motor rev
    public static final double GEAR_RATIO  = 1;  // motor:wheel upgear
    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    /**
     * Creates the launcher subsystem.
     *
     * @param hm Hardware map from OpMode
     */
    public Launcher(HardwareMap hm) {
        launcherLeft = new MotorEx(hm, "launcherL");
        launcherRight = new MotorEx(hm, "launcherR");

        shooterPid.setTolerance(25);
        shooterPid.setIntegrationBounds(-1, 1);

        launcherLeft.setInverted(false);
        launcherRight.setInverted(false);

        launcherLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        launcherRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        launcherLeft.setRunMode(Motor.RunMode.RawPower);
        launcherRight.setRunMode(Motor.RunMode.RawPower);

        launcherLeft.stopAndResetEncoder();

        disable();

        createLaunchTable();
    }

    // ------------- Public API -------------

    /** Enable closed-loop control and begin ramping toward the target RPM. */
    public void enable() {
        PARAMS.enabledPid = true;
        PARAMS.inTolStartNanos = 0L;
        PARAMS.isReadyToLaunch = false;
        shooterPid.reset();
        PARAMS.currentTargetRpm = getShooterRPM(); // ramp from current speed
    }

    /** Disable closed-loop control and stop the motor. */
    public void disable() {
        PARAMS.enabledPid = false;
        stop();
    }

    public void createLaunchTable () {
        for (double[] pair : LauncherTable) {
            treeMap.put(pair[0], pair[1]);
        }
    }


    /** @return true if PID control is enabled. */
    public boolean isEnabled() {
        return PARAMS.enabledPid;
    }
    public int ticker = 0;
    /** Directly set motor power (bypasses PID). */
    public void setPower(double power) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("SetPoser called amount", ++ticker);
        dashboard.sendTelemetryPacket(p);
        launcherLeft.set(power);
    }

    /** Immediately stop the motor and reset ready state. */
    public void stop() {
        launcherLeft.stopMotor();
        PARAMS.inTolStartNanos = 0L;
        PARAMS.isReadyToLaunch = false;
        PARAMS.currentTargetRpm = 0;
    }

    // ----------------- Target & tuning -----------------

    public void setTargetRpm(double rpm) { PARAMS.targetRpm = Math.max(0, rpm); }
    public double getTargetRpm() { return PARAMS.targetRpm; }

    public void getAndSetFlywheelByDistance () {
        Pose2d tagLocation = RobotState.getInstance().getTeam() ? blueTagPose : redTagPose;
        double distance = 100 * RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(tagLocation.getTranslation());
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance", distance - 17.78);
        packet.put("Rpm", treeMap.get(distance - 17.78));
        dashboard.sendTelemetryPacket(packet);
        setTargetRpm(treeMap.get(distance - 17.78));
    }

    public void setReadyToleranceRpm(double tol) { PARAMS.readyToleranceRpm = Math.max(0, tol); }
    public double getReadyToleranceRpm() { return PARAMS.readyToleranceRpm; }

    public void setReadyHoldTimeSeconds(double sec) { PARAMS.readyHoldTimeSeconds = Math.max(0, sec); }
    public double getReadyHoldTimeSeconds() { return PARAMS.readyHoldTimeSeconds; }

    // ----------------- Telemetry helpers -----------------

    public double getTicksPerSec()  { return launcherLeft.encoder.getRawVelocity(); }
    public double getShooterRPM()  { return (getTicksPerSec() * 60.0) / TICKS_PER_REV; }
    public boolean isReadyToLaunch() {return PARAMS.isReadyToLaunch;}

    // ------------- Main control loop -------------

    @Override
    public void periodic() {
        final long now = System.nanoTime();
        final double dt = (PARAMS.lastLoopNanos == 0L) ? 0.02 : (now - PARAMS.lastLoopNanos) / 1e9;
        PARAMS.lastLoopNanos = now;

        shooterPid.setPIDF(PARAMS.PIDKp, PARAMS.PIDKi, PARAMS.PIDKd, PARAMS.PIDKf);

        TelemetryPacket packet = new TelemetryPacket();

        if (PARAMS.enabledPid) {
            // 1) Ramp target to avoid brownouts
            final double maxStep = PARAMS.maxAccelRpmPerSec * dt;
            final double delta = PARAMS.targetRpm - PARAMS.currentTargetRpm;
            if (Math.abs(delta) > maxStep) {
                PARAMS.currentTargetRpm += Math.copySign(maxStep, delta);
            } else {
                PARAMS.currentTargetRpm = PARAMS.targetRpm;
            }

            // 2) Measure
            final double currentRpm = getShooterRPM();

            // 3) PID correction
            final double pidOut = shooterPid.calculate(currentRpm, PARAMS.currentTargetRpm);

            // 5) Apply
            double output = clamp(pidOut, 0.0, 1.0);
            packet.put("PID output", pidOut);
            packet.put("Output", output);

            // Set to pidOut needs to be tested what output but motor could never go negative
//            shooter.set(pidOut);
//            setPower(output);
            launcherLeft.set(output);
            launcherRight.set(output);
        }

//        shooter.set(.5);

        // --- Readiness logic ---
        double shooterErr = Math.abs(getShooterRPM() - PARAMS.targetRpm);
        boolean inTol = shooterErr <= PARAMS.readyToleranceRpm;

        if (inTol) {
            if (PARAMS.inTolStartNanos == 0L) PARAMS.inTolStartNanos = now;
            double heldSec = (now - PARAMS.inTolStartNanos) / 1e9;
            PARAMS.isReadyToLaunch = heldSec >= PARAMS.readyHoldTimeSeconds;
        } else {
            PARAMS.inTolStartNanos = 0L;
            PARAMS.isReadyToLaunch = false;
        }

        // Telemetry
        sendTelemetry(packet);
    }

    private void sendTelemetry (TelemetryPacket packet) {
        tele.addData("TgtRPM", PARAMS.targetRpm);
        tele.addData("CurTgt", PARAMS.currentTargetRpm);
        tele.addData("RPM", getShooterRPM());
        tele.addData("Ready", PARAMS.isReadyToLaunch);
        tele.addData("Enabled", PARAMS.enabledPid);
        tele.update();

        packet.put("TgtRPM", PARAMS.targetRpm);
        packet.put("CurTgt", PARAMS.currentTargetRpm);
        packet.put("RPM", getShooterRPM());
        packet.put("Ready", PARAMS.isReadyToLaunch);
        packet.put("Enabled", PARAMS.enabledPid);
        packet.put("Launcher encoder", launcherLeft.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
    }


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
