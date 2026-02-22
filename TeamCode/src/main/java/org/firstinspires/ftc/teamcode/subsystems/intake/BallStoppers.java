package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The HoldControl subsystem controls a positional servo responsible
 * for holding, loading, or releasing game elements (e.g. balls).
 *
 * <p>Now uses physical angle control via turnToAngle().
 */
public class BallStoppers implements Subsystem {

    private final ServoEx ballStopperLeft;
    private final ServoEx ballStopperRight;

    private BallTracker ballTracker;

    // === Servo configuration ===
    public static final double UP_ANGLE_LEFT = .5;
    public static final double DOWN_ANGLE_LEFT = .75;

    public static final double UP_ANGLE_RIGHT = .48;
    public static final double DOWN_ANGLE_RIGHT = .25;

    /**
     * Constructs the HoldControl subsystem.
     *
     * @param hm HardwareMap for servo initialization
     */
    public BallStoppers(HardwareMap hm) {
        ballStopperLeft = new SimpleServo(hm, "ballStopperLeft", 0, 1);
        ballStopperLeft.turnToAngle(UP_ANGLE_LEFT);
//        ballStopperLeft.turnToAngle(DOWN_ANGLE_LEFT);

        ballStopperRight = new SimpleServo(hm, "ballStopperRight", 0, 1);
//        ballStopperRight.turnToAngle(UP_ANGLE_RIGHT);
        ballStopperRight.turnToAngle(DOWN_ANGLE_RIGHT);

    }

    /**
     * Constructs the HoldControl subsystem.
     *
     * @param hm HardwareMap for servo initialization
     */
    public BallStoppers(HardwareMap hm, BallTracker ballTracker) {
        ballStopperLeft = new SimpleServo(hm, "ballStopperLeft", 0, 1);
        ballStopperLeft.turnToAngle(UP_ANGLE_LEFT);
//        ballStopperLeft.turnToAngle(DOWN_ANGLE_LEFT);

        ballStopperRight = new SimpleServo(hm, "ballStopperRight", 0, 1);
//        ballStopperRight.turnToAngle(UP_ANGLE_RIGHT);
        ballStopperRight.turnToAngle(DOWN_ANGLE_RIGHT);

        this.ballTracker = ballTracker;
    }

    /** Turns servo to the specified preset angle based on mode. */
    public void turnStopperLeft (double angle) {
        ballStopperLeft.turnToAngle(angle);
    }
    public void turnStopperRight (double angle) {
        ballStopperRight.turnToAngle(angle);
    }
    public void turnStopperBoth (double angleLeft, double angleRight) {
        turnStopperLeft(angleLeft);
        turnStopperRight(angleRight);
    }

    public boolean isStopperRightDown() {
        return ballStopperRight.getAngle() <= DOWN_ANGLE_RIGHT;
    }

    public boolean isStopperLeftDown () {
        return ballStopperLeft.getAngle() >= DOWN_ANGLE_LEFT;
    }




    @Override
    public void periodic() {
        if (ballTracker != null) {
            ballTracker.setGatesDown(isStopperLeftDown(), isStopperRightDown());
        } else {
            tele.addLine("Ball Tracker null stoppers");
        }
    }
}
