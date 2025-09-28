package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.util.BallColor;

public class Sorter implements Subsystem {

    // === Tuning constants (adjust on-robot) ===
    public static double CCW_POWER = -1;   // CCW is negative;
    public static double CW_POWER = 1;   // CW is positive;
    public static long STEP_MS = 200;    // time to advance exactly one slot in mills
    public static long FEED_MS  = 500;    // time to push exactly one ball into flywheel in mills
    public static int SLOT_COUNT   = 3;       // how many slots on the sorter

    private final CRServo sorterRotate;
    private int currentIndex;
    private BallColor[] slots = new BallColor[3];

//    private final RevColorSensorV3 colorSensorV3;

    private boolean ledEnabled;

    public Sorter(HardwareMap hm, int currentIndex, BallColor[] slots) {
        this.sorterRotate = hm.get(CRServo.class, "sorter");

        // Set the servo to to rotate

        // Optional but recommended: ensure a sensible PWM range for CR servos.
        if (sorterRotate instanceof CRServoImplEx) {
            ((CRServoImplEx) sorterRotate).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        sorterRotate.setPower(0);

        setCurrentIndex(currentIndex);

        setSlots(slots);

//        colorSensorV3 = hm.get(RevColorSensorV3.class, "colorSensor");
//        colorSensorV3.setGain(1);
//        setLed(true);
    }

    // === Inventory/Index helpers ===
    public int getCurrentIndex() { return currentIndex; }
    public BallColor getCurrentColor() { return slots[currentIndex]; }
    public BallColor[] getSlots() { return slots.clone(); }

    /** Set the current slot to the color given */
    public void setSlotNow(BallColor color) { slots[currentIndex] = color; }

    public double getPower() { return sorterRotate.getPower(); }


    public void setPower(double power) {
        sorterRotate.setPower(power);
    }

    public void setCurrentIndex (int index) {
        currentIndex = wrapIndex(index);
    }

    public void setSlots (BallColor[] newSlots) {
        slots = newSlots;
    }

    /**
     * Advances one slot in the direction given
     * @param direction 1 increase or -1 decrease
     * */
    public void advanceSlot (double direction) {
        currentIndex = wrapIndex(currentIndex + (int) Math.rint(direction));
    }

    /**
     * Takes the color you want and gives the closest slot index return -1 if none exist.
     * Will prioritize CCW for launcher preference
     * @param target the color you want to find
     * @return returns the index of the closest color -1 if none
     * */
    public int closestSlotToColor (BallColor target) {
        if (target == null || !target.isBall()) return -1;
        int copyCurrentIndex = currentIndex;
        for (int steps = 0; steps < SLOT_COUNT; steps++) {
            if (slots[copyCurrentIndex] == target) return copyCurrentIndex;
            copyCurrentIndex = wrapIndex(copyCurrentIndex - 1);
        }
        return -1;
    }

    /**
     * Takes the given index and returns the power needed to go to that index
     * @return -1 if index < currentIndex else 1
     * */
    public int getIndexOffset (int index) {
        if (index < currentIndex) {
            return -1;
        } else {
            return 1;
        }
    }


//    /**
//     * Turns the ball to the index using the closest path
//     * @param index the slot you want to turn to
//     * */
//    public void turnToIndex (int index) {
//        if (index == currentIndex) {
//            turnTo(0);
//        } else if (index == wrapIndex(currentIndex - 1)) {
//            turnToForTime(CCW_POWER, STEP_MS);
//        } else if (index == wrapIndex(currentIndex + 1)) {
//            turnToForTime(CW_POWER, STEP_MS);
//        }
//    }

    public boolean isLedEnabled() { return ledEnabled; }

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


//    public void setLed(boolean enable) {
//        if (colorSensorV3 instanceof SwitchableLight) {
//            ((SwitchableLight) colorSensorV3).enableLight(enable);
//        }
//        ledEnabled = enable;
//    }

    @Override
    public void periodic() {
        tele.addData("ServoPower", getPower());
    }

}