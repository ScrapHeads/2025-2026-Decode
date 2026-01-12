package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * The LauncherHood subsystem controls the adjustable hood servo
 * responsible for changing projectile launch angle.
 *
 * <p>This uses angle-based control via turnToAngle() for precise positioning.
 */
@Config
public class TurretHood implements Subsystem {

    /**
     * Holds all tunable parameters and control state for the launcher.
     * Marked public so FTC Dashboard can reflect values in the Config tab.
     */
    public static class Params {
        public double currentAngle = 0.0;

        public boolean autoAdjust = true;
    }

    /** Instance of params for this launcher. */
    public static Params PARAMS = new Params();


    private final ServoEx hoodServo;

    // Physical range of the servo
    public static final double MIN_ANGLE_SERVO = 500;
    public static final double MAX_ANGLE_SERVO = 2500;

    public static final double MAX_SHOOT_ANGLE = 2329;
    public static final double MIN_SHOOT_ANGLE = 744;

    /**
     * Creates a new LauncherHood subsystem.
     *
     * @param hm HardwareMap for accessing the servo
     */
    public TurretHood(HardwareMap hm) {
        // Axon MAX M2
        hoodServo = new SimpleServo(hm, "turretHood", MIN_ANGLE_SERVO, MAX_ANGLE_SERVO);
        setAngle(MIN_SHOOT_ANGLE);
    }

    /**
     * Sets the hood to a specific angle in degrees.
     *
     * @param angle The target angle (degrees)
     */
    public void setAngle(double angle) {
        // Clamp angle to physical servo limits

        double safeAngle = Math.max(MIN_SHOOT_ANGLE, Math.min(angle, MAX_SHOOT_ANGLE));
        hoodServo.turnToAngle(safeAngle);
        PARAMS.currentAngle = safeAngle;
    }

    /** @return The current logical hood angle (degrees). */
    public double getCurrentAngle() {
        return PARAMS.currentAngle;
    }

    public void enableAutoAdjust () {PARAMS.autoAdjust = true;}
    public void disableAutoAdjust () {PARAMS.autoAdjust = false;}

    @Override
    public void periodic() {
        // For testing only not normally not needed
//        setAngle(PARAMS.currentAngle);

        if (PARAMS.autoAdjust) {
            double distance = Constants.turretLookupTable.getDistance();
            setAngle(Constants.turretLookupTable.get(distance).hoodDeg);
        } else {
            hoodServo.turnToAngle(PARAMS.currentAngle);
        }

        tele.addData("Launcher Hood Angle", "%.1f°", hoodServo.getAngle());
    }
}
