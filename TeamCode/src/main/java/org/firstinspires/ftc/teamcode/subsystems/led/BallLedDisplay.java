// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/led/BallLedDisplaySubsystem.java
package org.firstinspires.ftc.teamcode.subsystems.led;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.intake.BallTracker;
import org.firstinspires.ftc.teamcode.util.PrismLedDriver;

import java.util.Arrays;

/**
 * Displays the BallLineTrackerSubsystem's 7-slot "line" on two 12-LED strips (24 LEDs total).
 *
 * Requirement mapping:
 *  - Each strip displays the full 7-slot array (0..6).
 *  - Second strip (LEDs 12..23) is mirrored so both sides "read" the same direction on the robot.
 *  - Colors:
 *      GREEN  -> green
 *      PURPLE -> purple
 *      UNKNOWN-> yellow
 *      EMPTY  -> white
 *  - If more than 3 balls are inside, highlight the 4th ball (in FEED_ORDER) as red.
 *
 * Hardware:
 *  - goBILDA Prism RGB LED Driver on I2C, named "ledStrip" in the Robot Configuration.
 */
public class BallLedDisplay implements Subsystem {

    public static class Params {
        // If colors look wrong, switch to GRB.
        public PrismLedDriver.PixelOrder pixelOrder = PrismLedDriver.PixelOrder.RGB;

        // Slot colors
        public int[] rgbGreen   = {0, 255, 0};
        public int[] rgbPurple  = {160, 0, 255};
        public int[] rgbUnknown = {255, 255, 0};     // yellow
        public int[] rgbEmpty   = {255, 255, 255};   // white

        // Over-capacity highlight (4th ball)
        public int[] rgbOverCap = {255, 0, 0};       // red

        // Unused LEDs (LEDs 7..11 on each strip, since we display 7 slots)
        public int[] rgbUnused  = {0, 0, 0};         // off
    }

    public static Params PARAMS = new Params();

    private final BallTracker tracker;
    private final PrismLedDriver prism;

    private static final int LED_COUNT = 24;

    // Frame is RGB bytes (R,G,B per LED) for 24 LEDs.
    private final byte[] frame = new byte[LED_COUNT * 3];
    private final byte[] lastFrame = new byte[LED_COUNT * 3];

    /**
     * Slot order for "consumption/priority" to identify the 4th ball.
     * Adjust this order if your actual usage order differs.
     */
    private static final int[] FEED_ORDER = new int[] {
            3, // CENTER
            2, // L_PATH
            4, // R_PATH
            1, // L_INNER
            5, // R_INNER
            0, // L_OUTER
            6  // R_OUTER
    };

    public BallLedDisplay(HardwareMap hm, BallTracker tracker) {
        this.tracker = tracker;
        this.prism = PrismLedDriver.fromHardwareMap(hm, "ledStrip");

        prism.setLedCountVerified(LED_COUNT);
        prism.clearAnimations();

        // Force first write
        Arrays.fill(lastFrame, (byte) 0x7F);
    }

    @Override
    public void periodic() {
        // 1) Fill everything with "unused"
        for (int led = 0; led < LED_COUNT; led++) {
            setLedRgb(frame, led, PARAMS.rgbUnused[0], PARAMS.rgbUnused[1], PARAMS.rgbUnused[2]);
        }

        // 2) Determine if we need an over-capacity highlight
        int ballCount = tracker.getBallCount();
        int overCapSlot = (ballCount > 3) ? findNthBallSlot(4) : -1;

        // 3) Paint slots 0..6 on both strips
        for (int slot = 0; slot <= 6; slot++) {
            int[] rgb = mapSlotColor(tracker.getSlotColor(slot));
            if (slot == overCapSlot) rgb = PARAMS.rgbOverCap;

            // Strip 1: LEDs 0..11 (we use 0..6)
            int ledStrip1 = slot;

            // Strip 2: LEDs 12..23 mirrored (so slot 0 is at the opposite end)
            int ledStrip2 = 12 + (11 - slot);

            setLedRgb(frame, ledStrip1, rgb[0], rgb[1], rgb[2]);
            setLedRgb(frame, ledStrip2, rgb[0], rgb[1], rgb[2]);
        }

        // 4) Only write if changed
        if (!Arrays.equals(frame, lastFrame)) {
            prism.writeCustomFrameRgb(frame, PARAMS.pixelOrder);
            System.arraycopy(frame, 0, lastFrame, 0, frame.length);
        }
    }

    private int[] mapSlotColor(BallTracker.BallColor c) {
        switch (c) {
            case GREEN:   return PARAMS.rgbGreen;
            case PURPLE:  return PARAMS.rgbPurple;
            case UNKNOWN: return PARAMS.rgbUnknown;
            case EMPTY:
            default:      return PARAMS.rgbEmpty;
        }
    }

    /**
     * Finds the slot index of the Nth ball in FEED_ORDER (1-based).
     * Returns -1 if fewer than N balls exist.
     */
    private int findNthBallSlot(int n) {
        int count = 0;
        for (int slot : FEED_ORDER) {
            if (tracker.isSlotOccupied(slot)) {
                count++;
                if (count == n) return slot;
            }
        }
        return -1;
    }

    private static void setLedRgb(byte[] buf, int ledIndex, int r, int g, int b) {
        int i = ledIndex * 3;
        buf[i]     = (byte) (r & 0xFF);
        buf[i + 1] = (byte) (g & 0xFF);
        buf[i + 2] = (byte) (b & 0xFF);
    }
}
