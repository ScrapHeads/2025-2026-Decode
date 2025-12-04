package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.util.BallColor.GREEN;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.RilLib.Control.PID.PIDController;
import org.firstinspires.ftc.teamcode.RilLib.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.Arrays;

/**
 * The Sorter subsystem handles the rotation of the sorting wheel that organizes balls
 * into slots based on detected color using a REV Color Sensor V3.
 *
 * <p>It supports:
 * <ul>
 *     <li>Continuous rotation servo for sorting wheel</li>
 *     <li>REV Color Sensor V3 for color classification</li>
 *     <li>Swift Robotics magnetic sensor for rotation tracking</li>
 * </ul>
 *
 * <p>Constants are adjustable and should be tuned on-robot.
 */
@Config
public class Sorter implements Subsystem {
    public static class Params {
        public double kp = 0.00017;
        public double ki = 0.0;
        public double kd = 0.0;
    }
    // === Tuning constants (adjust on-robot) ===
    public static final int CCW_DIRECTION = 1;   // CCW is negative
    public static final int CW_DIRECTION = -1;     // CW is positive
    public static final int SLOT_COUNT = 3;      // number of slots on the sorter

    private final RevColorSensorV3 colorSensorV3;
    private final DigitalChannel magneticSensor;
    private final MotorEx sorter;

    private int currentIndex;
    private BallColor[] slots = new BallColor[SLOT_COUNT];
    private boolean ledEnabled = false;

    //TODO find these values once the encoder is attached
    public final int TICKS_PER_THIRD_OF_TURN = 8231 / 3;
    private final int PID_TOLERANCE_IN_TICKS = 50;
    private final int INDEX_TOLERANCE_IN_TICKS = TICKS_PER_THIRD_OF_TURN / 2;

    public static Params PARAMS = new Params();

    public final PIDController pidController = new PIDController(PARAMS.kp, PARAMS.ki, PARAMS.kd);

    private double turnPos = 0;

    public boolean enabledPid = true;

    /**
     * Constructor initializes servo, color sensor, and magnetic sensor.
     *
     * @param hm           The HardwareMap used for device lookup
     */
    public Sorter(HardwareMap hm) {
        // === Initialize color sensor ===
        colorSensorV3 = hm.get(RevColorSensorV3.class, "colorSensor");
        colorSensorV3.setGain(10); // Gain range 1–60 depending on ambient light
        setLed(true);

        sorter = new MotorEx(hm, "sorter");
        sorter.stopAndResetEncoder();
        sorter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        sorter.setInverted(false);

        pidController.setTolerance(PID_TOLERANCE_IN_TICKS);
        pidController.reset();

        // === Initialize magnetic sensor ===
        magneticSensor = hm.get(DigitalChannel.class, "magSensor");
        magneticSensor.setMode(DigitalChannel.Mode.INPUT);

        setCurrentIndex(0);
        setSlots(RobotState.getInstance().getBallColors());
    }

    // === Inventory/Index helpers ===

    /** @return the current slot index */
    public int getCurrentIndex() { return currentIndex; }

    /** @return the BallColor currently under the color sensor */
    public BallColor getCurrentColor() { return slots[currentIndex]; }

    /** @return a copy of the slot array */
    public BallColor[] getSlots() { return slots.clone(); }

    /** Set the current slot color manually. */
    public void setSlotCurrent(BallColor color) { slots[currentIndex] = color; }

    /** @return current power level applied to the sorting servo */
    public double getPower() { return sorter.get(); }

    /** Sets the servo rotation power. */
    public void setPower(double power) { sorter.set(power); }

    /** Updates the current index (wrapped automatically). */
    public void setCurrentIndex(int index) { currentIndex = wrapIndex(index); }

    /** Sets the slot array. */
    public void setSlots(BallColor[] newSlots) { slots = newSlots; }

    public double getCurrentPos() {return sorter.getCurrentPosition();}
    public void resetCurrentPos() {
        sorter.resetEncoder();}

    public double getTurnPos () {return turnPos;}
    public void setTurnPos (double turnPos) {this.turnPos = turnPos;}

    public void setTurnPosWithOffset (double offset) {this.turnPos += offset;}

    public boolean isAtSetPoint () {
        return Math.abs(Math.abs(getCurrentPos()) - Math.abs(turnPos)) <= INDEX_TOLERANCE_IN_TICKS;
    }

    public int getIndex () {
        double pos = sorter.getCurrentPosition();
        double ticksPerSlot = TICKS_PER_THIRD_OF_TURN;

        // Compute fractional slot position
        double slotPosition = pos / ticksPerSlot;

        // Compute nearest integer slot
        int nearestSlot = (int) Math.round(slotPosition);

        // Compute the difference between actual pos and the nearest slot’s exact tick position
        double tickError = Math.abs(pos - (nearestSlot * ticksPerSlot));

        // Only snap if we're within tolerance
        if (tickError <= INDEX_TOLERANCE_IN_TICKS) {
            return Math.floorMod(nearestSlot, SLOT_COUNT);
        } else {
            // Stay at the previous slot if within the "dead zone"
            int floorSlot = (int) Math.floor(slotPosition);
            return Math.floorMod(floorSlot, SLOT_COUNT);
        }

//        double pos = encoder.getCurrentPosition();
//        return (int) Math.floorMod(Math.round(pos / TICKS_PER_THIRD_OF_TURN), SLOT_COUNT);
    }

    public int getSetIndex () {
        return Math.floorMod((int) turnPos, SLOT_COUNT);
    }


    public void turnOneSlotDirection (int direction) {
        // Only move if direction is non-zero
        if (direction == 0) return;

        // Determine new target position
        int offset = (int) (Math.signum(direction) * TICKS_PER_THIRD_OF_TURN);
        double newTarget = getTurnPos() + offset;

        // Update our internal goal
        setTurnPos(newTarget);
    }

    /**
     * Takes the color you want and gives the closest slot index.
     * Returns -1 if none exist.
     * Prioritizes CCW for launcher preference.
     *
     * @param target the color you want to find
     * @return the index of the closest color (-1 if none)
     */
    public int closestSlotToColor(BallColor target) {
        if (target == null || !target.isBall()) return -1;
        int copyCurrentIndex = currentIndex;
        for (int steps = 0; steps < SLOT_COUNT; steps++) {
            if (slots[copyCurrentIndex] == target) return copyCurrentIndex;
            copyCurrentIndex = wrapIndex(copyCurrentIndex - 1);
        }
        return -1;
    }

    public int findStartOffset(BallColor[] pattern, BallColor[] stored) {
        if (pattern == null || stored == null) return 0;
        int n = 3;

        // Compare pattern to each possible rotation of stored
        boolean match0 = true, matchForward = true, matchBackward = true;
        int ci = getSetIndex();
        for (int i = 0; i < n; i++) {
            // no rotation
            if (pattern[i] != stored[(i + ci) % n]) match0 = false;
            // rotated forward (+1)
            if (pattern[i] != stored[(i + ci + 1) % n]) matchForward = false;
            // rotated backward (-1)
            if (pattern[i] != stored[(i + ci - 1 + n) % n]) matchBackward = false;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Match0", match0);
        packet.put("MatchForward", matchForward);
        packet.put("MatchBackward", matchBackward);
        dashboard.sendTelemetryPacket(packet);

        if (match0) return 0;
        if (matchForward) return CCW_DIRECTION;
        if (matchBackward) return CW_DIRECTION;

        return 0; // default if nothing matches
    }

    /**
     * Takes the given index and returns the power direction
     * needed to go to that index.
     * @return -1 if index < currentIndex else 1
     */
    public int getTurnOffset(int index) {
        if (wrapIndex(currentIndex + 1) == index) {
            return -1;
        } else if (wrapIndex(currentIndex - 1) == index) {
            return 1;
        } else {
            return 0;
        }
    }

    // === Color Sensor ===

    /** @return true if the LED on the color sensor is enabled */
    public boolean isLedEnabled() { return ledEnabled; }

    /** Enable or disable the onboard LED for the color sensor. */
    public void setLed(boolean enable) {
        if (colorSensorV3 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensorV3).enableLight(enable);
        }
        ledEnabled = enable;
    }

    /**
     * Detects the color of the current ball using calibrated RGB thresholds
     * for GREEN and PURPLE colors. Thresholds should be tuned on-robot
     * for your lighting and sensor distance.
     *
     * @return the detected BallColor (GREEN, PURPLE, or EMPTY)
     */
    public BallColor detectBallColor() {
        int r = colorSensorV3.red();
        int g = colorSensorV3.green();
        int b = colorSensorV3.blue();

        // --- Normalize readings to minimize lighting variance ---
        double total = r + g + b;
        if (total == 0) return BallColor.EMPTY;

        double rNorm = r / total;
        double gNorm = g / total;
        double bNorm = b / total;

        // --- GREEN detection: strong green dominance ---
        if (gNorm > rNorm * 2.8 && gNorm > bNorm * 1.2) {
            return GREEN;
        }

        // --- PURPLE detection: red + blue high, green low ---
        double avgRB = (rNorm + bNorm) / 1.9;
        if (avgRB > gNorm) {
            return BallColor.PURPLE;
        }

        // --- None detected ---
        return BallColor.EMPTY;
    }

    // === Magnetic Sensor ===

    /**
     * Checks whether the Swift Robotics magnetic sensor has been triggered.
     *
     * <p>Typically, this sensor goes LOW (false) when it detects a magnet passing by.
     * The logic may need to be inverted depending on your sensor wiring.
     *
     * @return true if the sorter has completed one rotation trigger event.
     */
    public boolean isMagnetTriggered() {
        // Some sensors return LOW when triggered. Invert logic if needed.
        return !magneticSensor.getState();
    }

    // === Utility ===

    /**
     * Wrap an integer index into the valid range [0, SLOT_COUNT - 1].
     *
     * This makes the slot index behave like a circular buffer.
     * For SLOT_COUNT = 3:
     *   normalize(0)   -> 0
     *   normalize(1)   -> 1
     *   normalize(2)   -> 2
     *   normalize(3)   -> 0
     *   normalize(-1)  -> 2
     *
     * @param rawIndex the raw index (may be negative or too large)
     * @return a wrapped index within [0, SLOT_COUNT - 1]
     */
    private int wrapIndex(int rawIndex) {
        int m = rawIndex % SLOT_COUNT;
        return m < 0 ? m + SLOT_COUNT : m;
    }

    private void writeData () {
//        tele.addData("ServoPower", getPower());
        tele.addData("Sorter Index", getCurrentIndex());
        tele.addData("Detected Color", detectBallColor());
//        tele.addData("Sorter pos", getCurrentPos());
//        tele.addData("Turn pos", getTurnPos());
//        tele.addData("LED Enabled", isLedEnabled());
        tele.addData("Color Sensor (R,G,B)", "%d, %d, %d",
                colorSensorV3.red(), colorSensorV3.green(), colorSensorV3.blue());
        tele.addData("Slots", Arrays.toString(RobotState.getInstance().getBallColors()));
        tele.addData("Pattern", Arrays.toString(RobotState.getInstance().getPattern()));
        tele.addData("StartOffset", findStartOffset(RobotState.getInstance().getPattern(), RobotState.getInstance().getBallColors()));
//        tele.addData("Mag Sensor Triggered", isMagnetTriggered());
    }

    // === Periodic Telemetry ===
    @Override
    public void periodic() {
        if (isAtSetPoint() && getCurrentIndex() != getIndex()) {
            setCurrentIndex(getIndex());
        }
        double output = pidController.calculate(sorter.getCurrentPosition(), turnPos);

        BallColor seenColor = detectBallColor();

        if (seenColor != getCurrentColor() && isAtSetPoint() && Math.abs(output) < .04) {
            setSlotCurrent(seenColor);
        }

        TelemetryPacket p = new TelemetryPacket();
        p.put("Sorter output", output);
        dashboard.sendTelemetryPacket(p);

        RobotState.getInstance().setBallColors(slots);

        if (enabledPid) {
            sorter.set(-output);
        }

        writeData();
    }
}
