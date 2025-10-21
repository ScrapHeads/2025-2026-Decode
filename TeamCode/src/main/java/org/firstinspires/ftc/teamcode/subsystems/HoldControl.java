package org.firstinspires.ftc.teamcode.subsystems;

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
public class HoldControl implements Subsystem {

    private final ServoEx holdServo;

    // === Servo configuration ===
    public static final double MIN_ANGLE = 500.0;
    public static final double MAX_ANGLE = 2500.0;

    // === Servo angle presets (in degrees) ===
    public static final double TRANSPORT_ANGLE = 1500;
    public static final double LOADING_ANGLE   = 2036;
    public static final double LAUNCHING_ANGLE = 900;

    private HoldPosition currentPosition = HoldPosition.TRANSPORT;

    /** The functional modes for the hold control servo. */
    public enum HoldPosition {
        TRANSPORT,
        LOADING,
        LAUNCHING
    }

    /**
     * Constructs the HoldControl subsystem.
     *
     * @param hm HardwareMap for servo initialization
     */
    public HoldControl(HardwareMap hm) {
        // Axon MAX M2 supports 270° travel – specify full range
        holdServo = new SimpleServo(hm, "holdControl", MIN_ANGLE, MAX_ANGLE);
        holdServo.turnToAngle(TRANSPORT_ANGLE);
    }

    /** Turns servo to the specified preset angle based on mode. */
    public void moveTo(HoldPosition position) {
        switch (position) {
            case TRANSPORT:
                holdServo.turnToAngle(TRANSPORT_ANGLE);
                break;
            case LOADING:
                holdServo.turnToAngle(LOADING_ANGLE);
                break;
            case LAUNCHING:
                holdServo.turnToAngle(LAUNCHING_ANGLE);
                break;
        }
        currentPosition = position;
    }

    public HoldPosition getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public void periodic() {
        tele.addData("HoldControl State", currentPosition);
        tele.addData("Servo Angle", "%.1f°", holdServo.getAngle());
    }
}
