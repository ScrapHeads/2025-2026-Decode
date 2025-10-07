package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

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
public class LauncherHood implements Subsystem {

    private final ServoEx hoodServo;

    // === Physical range of the servo (degrees) ===
    public static final double MIN_ANGLE = 500;
    public static final double MAX_ANGLE = 2500;

    public static final double MAX_SHOOT_ANGLE = 1585;
    public static final double MIN_SHOOT_ANGLE = 1300;


    private double currentAngle = 0.0;

    /**
     * Creates a new LauncherHood subsystem.
     *
     * @param hm HardwareMap for accessing the servo
     */
    public LauncherHood(HardwareMap hm) {
        // Axon MAX M2 compatible (270° travel range)
        hoodServo = new SimpleServo(hm, "hood", MIN_ANGLE, MAX_ANGLE);
        hoodServo.turnToAngle(1485); // Start at 0° (lowest hood position)
    }

    /**
     * Sets the hood to a specific angle in degrees.
     *
     * @param angle The target angle (degrees)
     */
    public void setAngle(double angle) {
        // Clamp angle to physical servo limits
        double safeAngle = Math.max(MIN_SHOOT_ANGLE, Math.min(MAX_SHOOT_ANGLE, angle));
        hoodServo.turnToAngle(safeAngle);
        currentAngle = safeAngle;
    }

    /** @return The current logical hood angle (degrees). */
    public double getCurrentAngle() {
        return currentAngle;
    }

    @Override
    public void periodic() {
        tele.addData("Launcher Hood Angle", "%.1f°", hoodServo.getAngle());
    }
}
