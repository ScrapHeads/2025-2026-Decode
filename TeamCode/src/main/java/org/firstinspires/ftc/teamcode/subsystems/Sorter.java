package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.util.BallColor.GREEN;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.RilLib.Control.PID.PIDController;
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
public class Sorter implements Subsystem {

    // === Tuning constants (adjust on-robot) ===
    public static final int CCW_DIRECTION = -1;   // CCW is negative
    public static final int CW_DIRECTION = 1;     // CW is positive
    public static final long STEP_MS = 200;      // time to advance exactly one slot (ms)
    public static final int SLOT_COUNT = 3;      // number of slots on the sorter

    private final CRServo sorterRotate;
    private final RevColorSensorV3 colorSensorV3;
    private final DigitalChannel magneticSensor;
    private final MotorEx encoder;

    private int currentIndex;
    private BallColor[] slots = new BallColor[SLOT_COUNT];
    private boolean ledEnabled = false;

    //TODO find these values once the encoder is attached
    public final int TICKS_PER_THIRD_OF_TURN = 1;
    private final int TURN_TOLERANCE_IN_TICKS = 0;

    private double turnPos = 0;

    //TODO tune the pidController
    private final PIDController pidController = new PIDController(0, 0, 0);

    /**
     * Constructor initializes servo, color sensor, and magnetic sensor.
     *
     * @param hm           The HardwareMap used for device lookup
     */
    public Sorter(HardwareMap hm) {
        this.sorterRotate = hm.get(CRServo.class, "sorter");

        sorterRotate.setPower(0);

        // === Initialize color sensor ===
        colorSensorV3 = hm.get(RevColorSensorV3.class, "colorSensor");
        colorSensorV3.setGain(2); // Gain range 1–60 depending on ambient light
        setLed(true);

        encoder = new MotorEx(hm, "encoder");
        encoder.resetEncoder();

        pidController.setTolerance(2);

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
    public double getPower() { return sorterRotate.getPower(); }

    /** Sets the servo rotation power. */
    public void setPower(double power) { sorterRotate.setPower(power); }

    /** Updates the current index (wrapped automatically). */
    public void setCurrentIndex(int index) { currentIndex = wrapIndex(index); }

    /** Sets the slot array. */
    public void setSlots(BallColor[] newSlots) { slots = newSlots; }

    public double getCurrentPos() {return encoder.getCurrentPosition();}
    public void resetCurrentPos() {encoder.resetEncoder();}

    public double getTurnPos () {return turnPos;}
    public void setTurnPos (double turnPos) {this.turnPos = turnPos;}

    public void setTurnPosWithOffset (double offset) {this.turnPos += offset;}

    public boolean isAtSetPoint () {
        return Math.abs(Math.abs(getCurrentPos()) - Math.abs(turnPos)) <= TURN_TOLERANCE_IN_TICKS;
    }

    public int getIndex () {
        double pos = encoder.getCurrentPosition();
        return (int) Math.floorMod(Math.round(pos / TICKS_PER_THIRD_OF_TURN), SLOT_COUNT);
    }

    public void turnOneSlotDirection (int direction) {
        int offset = 0;
        if (direction > 0) {
            offset = TICKS_PER_THIRD_OF_TURN;
        } else if (direction < 0) {
            offset = -TICKS_PER_THIRD_OF_TURN;
        }
        setTurnPosWithOffset(offset);
    }

    /**
     * Advances one slot in the direction given
     * @param direction 1 increase or -1 decrease
     */
//    public void advanceSlot(double direction) {
//        if (direction > 0) {
//            direction = 1;
//        } else if (direction < 0) {
//            direction = -1;
//        }
//        currentIndex = wrapIndex(currentIndex + (int) direction);
//    }

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

    public int findStartIndex (BallColor[] pattern, BallColor[] stored) {
        if (Arrays.equals(pattern, stored)) {
            return 0;
        }

        int patternsGreen = 0;
        int storedGreen = 0;

        for (int i = 0; i < pattern.length; i++) {
            if (pattern[i] == GREEN) {
                patternsGreen = i;
            }
            if (stored[i] == GREEN) {
                storedGreen = i;
            }
        }

        int offset = 0;

        if (patternsGreen == 2) {
            offset = 1;
        } else if (patternsGreen == 1) {
            offset = -1;
        }

        return wrapIndex(storedGreen + offset);
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
        if (gNorm > rNorm * 1.3 && gNorm > bNorm * 1.3) {
            return GREEN;
        }

        // --- PURPLE detection: red + blue high, green low ---
        double avgRB = (rNorm + bNorm) / 2.0;
        if (avgRB > gNorm * 1.1) {
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
        return magneticSensor.getState();
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
        tele.addData("ServoPower", getPower());
        tele.addData("Sorter Index", getCurrentIndex());
        tele.addData("Sorter pos", getCurrentPos());
        tele.addData("LED Enabled", isLedEnabled());
        tele.addData("Color Sensor (R,G,B)", "%d, %d, %d",
                colorSensorV3.red(), colorSensorV3.green(), colorSensorV3.blue());
        tele.addData("Detected Color", detectBallColor());
        tele.addData("Mag Sensor Triggered", isMagnetTriggered());
    }

    // === Periodic Telemetry ===
    @Override
    public void periodic() {

        if (isMagnetTriggered() && isAtSetPoint() && getCurrentPos() != 0) {
            resetCurrentPos();
            setTurnPos(0);
        }

        if (isAtSetPoint() && getCurrentIndex() != getIndex()) {
            setCurrentIndex(getIndex());
        }

        if (detectBallColor() != getCurrentColor() && isAtSetPoint()) {
            setSlotCurrent(detectBallColor());
        }

        double output = pidController.calculate(encoder.getCurrentPosition(), turnPos);

        sorterRotate.setPower(output);

        writeData();
    }
}
