package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

/**
 * BallLineTrackerSubsystem
 *
 * One continuous "line" model with 7 slots indexed 0..6.
 *
 * End pockets (2-wide, side-by-side):
 *   Left pocket:  [0 outer] [1 inner]
 *   Right pocket: [5 inner] [6 outer]
 *
 * Single-file toward center:
 *   [2] left path -> center
 *   [4] right path -> center
 *
 * Center / feeder pickup (no sensor):
 *   [3]
 *
 * Sensor naming (RevColorSensorV3):
 *   Left pocket:  colorLeftFront,  colorLeftBack
 *   Right pocket: colorRightFront, colorRightBack
 *   Center path:  colorCenterLeft, colorCenterRight
 */
@Config
public final class BallTracker implements Subsystem {

    // -------------------- Tunable Parameters --------------------

    public static class Params {
        // Presence thresholds (mm)
        public double presentPocketMm = 50.0; // pocket Rev V3 distance thresholds
        public double presentCenterMm = 30; // center Rev V3 distance thresholds

        // Color classification
        public double minTotalRgb = 40.0;
        public double greenDominance = 2.5;
        public double greenOverBlue = 1.2;
        public double purpleMargin = 0.025;

        // Motion thresholds
        public double powerDeadband = 0.05;

        // Optional: inference timeout for "in transit" states, in seconds
        public double inferenceTimeoutSec = 0.9;

        // Debug
        public boolean debugDashboard = true;
    }

    public static Params PARAMS = new Params();

    // -------------------- Types --------------------

    public enum BallColor {
        EMPTY,
        UNKNOWN,
        GREEN,
        PURPLE
    }

    public static final class Ball {
        public final int id;
        public BallColor color = BallColor.UNKNOWN;
        public double lastSeenSec;

        private Ball(int id) { this.id = id; }

        @NonNull
        @Override
        public String toString() {
            return id + ":" + color;
        }
    }

    // -------------------- Hardware --------------------

    private final RevColorSensorV3 colorLeftFront;
    private final RevColorSensorV3 colorLeftBack;

    private final RevColorSensorV3 colorRightFront;
    private final RevColorSensorV3 colorRightBack;

    private final RevColorSensorV3 colorCenterLeft;
    private final RevColorSensorV3 colorCenterRight;

    // -------------------- Inputs from other systems --------------------

    private double intakeLeftPower = 0.0;
    private double intakeRightPower = 0.0;
    private double feederPower = 0.0;

    private boolean leftGateDown = false;
    private boolean rightGateDown = false;

    /** Call this every loop from your intake subsystem/commands. */
    public void setIntakePowers(double left, double right) {
        this.intakeLeftPower = left;
        this.intakeRightPower = right;
    }
    /** Call this every loop from your intake subsystem/commands. */
    public void setRightIntakePower(double right) {
        this.intakeRightPower = right;
    }
    /** Call this every loop from your intake subsystem/commands. */
    public void setLeftIntakePower(double left) {
        this.intakeLeftPower = left;
    }

    /** Call this every loop from your feeder/shooter subsystem/commands. */
    public void setFeederPower(double feederPower) {
        this.feederPower = feederPower;
    }

    /** Call this every loop or whenever state changes. down=true means blocked. */
    public void setGatesDown(boolean leftGateDown, boolean rightGateDown) {
        this.leftGateDown = leftGateDown;
        this.rightGateDown = rightGateDown;
    }
    /** Call this every loop or whenever state changes. down=true means blocked. */
    public void setRightGatesDown(boolean rightGateDown) {
        this.rightGateDown = rightGateDown;
    }
    /** Call this every loop or whenever state changes. down=true means blocked. */
    public void setLeftGateDown(boolean leftGateDown) {
        this.leftGateDown = leftGateDown;
    }

    // -------------------- Internal State --------------------

    private final Ball[] line = new Ball[7];
    private int nextId = 1;

    // Presence edges
    private final Edge pocketLeftEdge = new Edge();
    private final Edge pocketRightEdge = new Edge();
    private final Edge centerLeftEdge = new Edge();
    private final Edge centerRightEdge = new Edge();

    // Optional transit tracking (helps debugging / timeouts)
    private double leftTransitStartSec = -1.0;
    private double rightTransitStartSec = -1.0;

    public BallTracker(HardwareMap hm) {
        colorLeftFront = hm.get(RevColorSensorV3.class, "colorLeftFront");
        colorLeftBack = hm.get(RevColorSensorV3.class, "colorLeftBack");

        colorRightFront = hm.get(RevColorSensorV3.class, "colorRightFront");
        colorRightBack = hm.get(RevColorSensorV3.class, "colorRightBack");

        colorCenterLeft = hm.get(RevColorSensorV3.class, "colorCenterLeft");
        colorCenterRight = hm.get(RevColorSensorV3.class, "colorCenterRight");
    }

    // -------------------- Slot Indices --------------------

    private static final int L_OUTER = 0;
    private static final int L_INNER = 1;
    private static final int L_PATH  = 2;

    private static final int CENTER  = 3;

    private static final int R_PATH  = 4;
    private static final int R_INNER = 5;
    private static final int R_OUTER = 6;

    // -------------------- Periodic --------------------

    @Override
    public void periodic() {
        double nowSec = TimeTracker.getTime();

        // Read pocket sensors (COLOR ONLY)
        BallColor lFrontColor = detectBallColor(colorLeftFront, true);
        BallColor lBackColor  = detectBallColor(colorLeftBack,  true);

        BallColor rFrontColor = detectBallColor(colorRightFront, true);
        BallColor rBackColor  = detectBallColor(colorRightBack,  true);

        // Presence based on color classification (not EMPTY)
        boolean leftPocketPresent =
                (lFrontColor != BallColor.EMPTY) || (lBackColor != BallColor.EMPTY);
        boolean rightPocketPresent =
                (rFrontColor != BallColor.EMPTY) || (rBackColor != BallColor.EMPTY);

        // Read center sensors (COLOR ONLY)
        BallColor cLeftColor  = detectBallColor(colorCenterLeft,  true);
        BallColor cRightColor = detectBallColor(colorCenterRight, true);

        boolean centerLeftPresent  = (cLeftColor  != BallColor.EMPTY);
        boolean centerRightPresent = (cRightColor != BallColor.EMPTY);

        // Update edges (distance unused)
        pocketLeftEdge.update(leftPocketPresent, Double.NaN);
        pocketRightEdge.update(rightPocketPresent, Double.NaN);
        centerLeftEdge.update(centerLeftPresent, Double.NaN);
        centerRightEdge.update(centerRightPresent, Double.NaN);

        // 1) Pocket arrivals
        if (pocketLeftEdge.rising) addIntoLeftPocket(nowSec);
        if (pocketRightEdge.rising) addIntoRightPocket(nowSec);

        // 2) Update pocket colors (best-effort; one sensor cannot fully disambiguate two balls)
        updatePocketColorLeft(nowSec,
                new Station(leftPocketPresent, Double.NaN, (lFrontColor != BallColor.EMPTY) ? lFrontColor : lBackColor),
                new Station(leftPocketPresent, Double.NaN, (lBackColor  != BallColor.EMPTY) ? lBackColor  : lFrontColor));

        updatePocketColorRight(nowSec,
                new Station(rightPocketPresent, Double.NaN, (rFrontColor != BallColor.EMPTY) ? rFrontColor : rBackColor),
                new Station(rightPocketPresent, Double.NaN, (rBackColor  != BallColor.EMPTY) ? rBackColor  : rFrontColor));

        // 3) Push inward from each side (pocket -> path)
        if (canPushLeftInward()) {
            pushFromLeftTowardCenter(nowSec);
        }
        if (canPushRightInward()) {
            pushFromRightTowardCenter(nowSec);
        }

        // 4) Move into center based on intake power AND gate state AND center-side sensor presence
        Station cLeftStation = new Station(centerLeftPresent, Double.NaN, cLeftColor);
        Station cRightStation = new Station(centerRightPresent, Double.NaN, cRightColor);
        tryMoveIntoCenter(nowSec, cLeftStation, cRightStation);

        // 5) Feeder consumes center (no sensor in center)
        if (isForward(feederPower) && line[CENTER] != null) {
            line[CENTER] = null;
        }

        // 6) Optional: clear stale transit flags
        if (leftTransitStartSec > 0 && (nowSec - leftTransitStartSec) > PARAMS.inferenceTimeoutSec) {
            leftTransitStartSec = -1.0;
        }
        if (rightTransitStartSec > 0 && (nowSec - rightTransitStartSec) > PARAMS.inferenceTimeoutSec) {
            rightTransitStartSec = -1.0;
        }

        // Telemetry
        tele.addData("Line", formatLine());
        tele.addData("Gates", "L=%s R=%s", leftGateDown ? "DOWN" : "UP", rightGateDown ? "DOWN" : "UP");
        tele.addData("intakeL", "%.2f", intakeLeftPower);
        tele.addData("intakeR", "%.2f", intakeRightPower);
        tele.addData("feeder", "%.2f", feederPower);

        tele.addData("LF", "color=%s", lFrontColor);
        tele.addData("LB", "color=%s", lBackColor);
        tele.addData("RF", "color=%s", rFrontColor);
        tele.addData("RB", "color=%s", rBackColor);
        tele.addData("cL", "present=%s color=%s", centerLeftPresent, cLeftColor);
        tele.addData("cR", "present=%s color=%s", centerRightPresent, cRightColor);

        if (PARAMS.debugDashboard) {
            TelemetryPacket p = new TelemetryPacket();
            for (int i = 0; i < line.length; i++) p.put("slot" + i, slotString(i));
            dashboard.sendTelemetryPacket(p);
        }

    }

    // -------------------- Movement Permissions --------------------

    private boolean canPushLeftInward() {
        return isForward(intakeLeftPower);
    }

    private boolean canPushRightInward() {
        return isForward(intakeRightPower);
    }

    private boolean canLeftEnterCenter() {
        return isForward(intakeLeftPower) && !leftGateDown;
    }

    private boolean canRightEnterCenter() {
        return isForward(intakeRightPower) && !rightGateDown;
    }

    // -------------------- Core Logic --------------------

    private void addIntoLeftPocket(double nowSec) {
        if (line[L_OUTER] == null) {
            line[L_OUTER] = newBall(nowSec);
        } else if (line[L_INNER] == null) {
            line[L_INNER] = newBall(nowSec);
        }
    }

    private void addIntoRightPocket(double nowSec) {
        if (line[R_OUTER] == null) {
            line[R_OUTER] = newBall(nowSec);
        } else if (line[R_INNER] == null) {
            line[R_INNER] = newBall(nowSec);
        }
    }

    /**
     * Left pocket pushes inward into [2].
     * Preference: INNER -> PATH first (ball closer to center should advance first).
     */
    private void pushFromLeftTowardCenter(double nowSec) {
        if (line[L_PATH] != null) return;

        if (line[L_INNER] != null) {
            line[L_PATH] = pop(L_INNER, nowSec);
            leftTransitStartSec = nowSec;
            return;
        }

        if (line[L_OUTER] != null) {
            line[L_PATH] = pop(L_OUTER, nowSec);
            leftTransitStartSec = nowSec;

            // After moving OUTER, slide remaining OUTER->INNER
            if (line[L_INNER] == null && line[L_OUTER] != null) {
                line[L_INNER] = pop(L_OUTER, nowSec);
            }
        }
    }

    /**
     * Right pocket pushes inward into [4].
     * Preference: INNER -> PATH first (ball closer to center should advance first).
     */
    private void pushFromRightTowardCenter(double nowSec) {
        if (line[R_PATH] != null) return;

        if (line[R_INNER] != null) {
            line[R_PATH] = pop(R_INNER, nowSec);
            rightTransitStartSec = nowSec;
            return;
        }

        if (line[R_OUTER] != null) {
            line[R_PATH] = pop(R_OUTER, nowSec);
            rightTransitStartSec = nowSec;

            // Slide 6->5 if possible
            if (line[R_INNER] == null && line[R_OUTER] != null) {
                line[R_INNER] = pop(R_OUTER, nowSec);
            }
        }
    }

    /**
     * Center movement is based on:
     * - intake power for that side
     * - that side gate NOT down
     * - that side center sensor sees presence
     * - corresponding path slot is occupied (2 for left, 4 for right)
     */
    private void tryMoveIntoCenter(double nowSec, Station cLeft, Station cRight) {
        if (line[CENTER] != null) return;

        boolean leftReady = canLeftEnterCenter() && cLeft.present && line[L_PATH] != null;
        boolean rightReady = canRightEnterCenter() && cRight.present && line[R_PATH] != null;

        if (!leftReady && !rightReady) return;

        if (leftReady && !rightReady) {
            line[CENTER] = pop(L_PATH, nowSec);
            applyColorIfKnown(line[CENTER], cLeft.color, nowSec);
            leftTransitStartSec = -1.0;
            return;
        }

        if (rightReady && !leftReady) {
            line[CENTER] = pop(R_PATH, nowSec);
            applyColorIfKnown(line[CENTER], cRight.color, nowSec);
            rightTransitStartSec = -1.0;
            return;
        }

        // Both ready: prefer higher magnitude intake power
        if (Math.abs(intakeLeftPower) >= Math.abs(intakeRightPower)) {
            line[CENTER] = pop(L_PATH, nowSec);
            applyColorIfKnown(line[CENTER], cLeft.color, nowSec);
            leftTransitStartSec = -1.0;
        } else {
            line[CENTER] = pop(R_PATH, nowSec);
            applyColorIfKnown(line[CENTER], cRight.color, nowSec);
            rightTransitStartSec = -1.0;
        }
    }

    // -------------------- Pocket Color Updates --------------------

    private void updatePocketColorLeft(double nowSec, Station front, Station back) {
        Ball target = null;
        if (line[L_INNER] != null) target = line[L_INNER];
        else if (line[L_OUTER] != null) target = line[L_OUTER];

        if (target == null) return;

        if (front.present) applyColorIfKnown(target, front.color, nowSec);
        else if (back.present) applyColorIfKnown(target, back.color, nowSec);
    }

    private void updatePocketColorRight(double nowSec, Station front, Station back) {
        Ball target = null;
        if (line[R_INNER] != null) target = line[R_INNER];
        else if (line[R_OUTER] != null) target = line[R_OUTER];

        if (target == null) return;

        if (front.present) applyColorIfKnown(target, front.color, nowSec);
        else if (back.present) applyColorIfKnown(target, back.color, nowSec);
    }

    // -------------------- Sensors + Color Detection --------------------

    private static final class Station {
        final boolean present;
        final double distanceMm;
        final BallColor color;

        Station(boolean present, double distanceMm, BallColor color) {
            this.present = present;
            this.distanceMm = distanceMm;
            this.color = color;
        }
    }

    private Station readRevStation(RevColorSensorV3 sensor, double presentMm) {
        double d = sensor.getDistance(DistanceUnit.MM);
        boolean present = !Double.isNaN(d) && d <= presentMm;
        BallColor c = detectBallColor(sensor, present);
        return new Station(present, d, c);
    }

    /**
     * Presence is authoritative:
     * - If not present => EMPTY
     * - If present but cannot classify => UNKNOWN
     */
    private BallColor detectBallColor(RevColorSensorV3 sensor, boolean present) {
//        if (!present) return BallColor.EMPTY;

        TelemetryPacket p = new TelemetryPacket();

        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        p.put("r", r);
        p.put("g", g);
        p.put("b", b);

        double total = (double) r + g + b;
        if (total < PARAMS.minTotalRgb) return BallColor.UNKNOWN;

        double rNorm = r / total;
        double gNorm = g / total;
        double bNorm = b / total;

        p.put("rNorm", rNorm);
        p.put("gNorm", gNorm);
        p.put("bNorm", bNorm);
        dashboard.sendTelemetryPacket(p);

        if (gNorm > rNorm * PARAMS.greenDominance && gNorm > bNorm * PARAMS.greenOverBlue) {
            return BallColor.GREEN;
        }

        double avgRB = (rNorm + bNorm) / 1.5;
        if ((avgRB - gNorm) > PARAMS.purpleMargin) {
            return BallColor.PURPLE;
        }

        return BallColor.EMPTY;
    }

    // -------------------- Utilities --------------------

    private Ball newBall(double nowSec) {
        Ball b = new Ball(nextId++);
        b.lastSeenSec = nowSec;
        return b;
    }

    private Ball pop(int idx, double nowSec) {
        Ball b = line[idx];
        line[idx] = null;
        if (b != null) b.lastSeenSec = nowSec;
        return b;
    }

    private void applyColorIfKnown(Ball b, BallColor sensed, double nowSec) {
        if (b == null) return;
        if (sensed == BallColor.GREEN || sensed == BallColor.PURPLE) {
            b.color = sensed;
            b.lastSeenSec = nowSec;
        }
    }

    private boolean isForward(double p) {
        return p > PARAMS.powerDeadband;
    }

    private static double minValid(double a, double b) {
        boolean aOk = !Double.isNaN(a);
        boolean bOk = !Double.isNaN(b);
        if (aOk && bOk) return Math.min(a, b);
        if (aOk) return a;
        if (bOk) return b;
        return Double.NaN;
    }

    private String formatLine() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < line.length; i++) {
            sb.append('[').append(i).append(':').append(slotString(i)).append(']');
            if (i < line.length - 1) sb.append(' ');
        }
        return sb.toString();
    }

    private String slotString(int i) {
        Ball b = line[i];
        return (b == null) ? "EMPTY" : b.toString();
    }

    // -------------------- Edge Helper --------------------

    private static final class Edge {
        boolean prevOcc = false;
        boolean rising = false;
        boolean falling = false;

        double prevDist = Double.NaN;
        double dist = Double.NaN;

        void update(boolean occNow, double distNow) {
            rising = !prevOcc && occNow;
            falling = prevOcc && !occNow;
            prevOcc = occNow;

            prevDist = dist;
            dist = distNow;
        }

        double deltaDist() {
            if (Double.isNaN(prevDist) || Double.isNaN(dist)) return 0.0;
            return dist - prevDist;
        }
    }

    // -------------------- Public Accessors (LED display, etc.) --------------------

    /** Returns the current ball color for a slot. EMPTY if no ball object in that slot. */
    public BallColor getSlotColor(int slotIdx) {
        if (slotIdx < 0 || slotIdx >= line.length) return BallColor.EMPTY;
        Ball b = line[slotIdx];
        return (b == null) ? BallColor.EMPTY : b.color;
    }

    /** Returns true if a ball exists in that slot. */
    public boolean isSlotOccupied(int slotIdx) {
        if (slotIdx < 0 || slotIdx >= line.length) return false;
        return line[slotIdx] != null;
    }

    /** Returns total balls currently tracked (occupied slots). */
    public int getBallCount() {
        int c = 0;
        for (Ball b : line) if (b != null) c++;
        return c;
    }
}