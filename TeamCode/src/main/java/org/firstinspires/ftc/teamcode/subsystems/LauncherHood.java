package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The LauncherHood subsystem controls the adjustable hood servo
 * responsible for changing projectile launch angle.
 *
 * <p>This uses angle-based control via turnToAngle() for precise positioning.
 */
@Config
public class LauncherHood implements Subsystem {

    /**
     * Holds all tunable parameters and control state for the launcher.
     * Marked public so FTC Dashboard can reflect values in the Config tab.
     */
    public static class Params {
        public double currentAngle = 0.0;
    }

    /** Instance of params for this launcher. */
    public static Params PARAMS = new Params();


    private final ServoEx hoodServo;

    // === Physical range of the servo (degrees) ===
    public static final double MIN_ANGLE = 500;
    public static final double MAX_ANGLE = 2500;

    public static final double HIGH_SHOOT_ANGLE = 1400;
    public static final double MID_SHOOT_ANGLE = 1620;
    public static final double AUTO_CLOSE_ANGLE = 1600;
    public static final double LOW_SHOOT_ANGLE = 1670;

    /**
     * Creates a new LauncherHood subsystem.
     *
     * @param hm HardwareMap for accessing the servo
     */
    public LauncherHood(HardwareMap hm) {
        // Axon MAX M2 compatible (270° travel range)
        hoodServo = new SimpleServo(hm, "hood", MIN_ANGLE, MAX_ANGLE);
//        hoodServo.turnToAngle(1540); // Start at LOW_SHOOT_ANGLE (lowest hood position)
        setAngle(1430);
    }


    /**
     * Sets the hood to a specific angle in degrees.
     *
     * @param angle The target angle (degrees)
     */
    public void setAngle(double angle) {
        // Clamp angle to physical servo limits
        double safeAngle = Math.min(LOW_SHOOT_ANGLE, Math.max(HIGH_SHOOT_ANGLE, angle));
        hoodServo.turnToAngle(safeAngle);
        PARAMS.currentAngle = safeAngle;
    }

    /** @return The current logical hood angle (degrees). */
    public double getCurrentAngle() {
        return PARAMS.currentAngle;
    }

    @Override
    public void periodic() {
        setAngle(getCurrentAngle());
        tele.addData("Launcher Hood Angle", "%.1f°", hoodServo.getAngle());
    }
}
