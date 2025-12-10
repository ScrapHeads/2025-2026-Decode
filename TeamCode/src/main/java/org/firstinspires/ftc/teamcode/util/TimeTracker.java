package org.firstinspires.ftc.teamcode.util;

public class TimeTracker {

    private static double timeOffset = 0;

    public static void setOffset() {
        timeOffset = 0;
        timeOffset = getTime();
    }

    /**
     * Get the robot time in seconds
     * @return The robot time in seconds
     */
    public static double getTime() {
        return (System.currentTimeMillis() / 1000.0) - timeOffset;
    }

    public static double convertTime(double time) {
        return time - timeOffset;
    }
}
