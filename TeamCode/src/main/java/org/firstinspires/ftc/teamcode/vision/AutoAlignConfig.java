package org.firstinspires.ftc.teamcode.vision;

/**
 * Configuration values for automatic vision-based alignment.
 * <p>
 * This class holds tuning constants and targets used when
 * driving the robot to align with a detected tag or object.
 * It includes PID-like proportional gains for strafe, forward,
 * and heading correction, as well as pixel tolerances for
 * acceptable alignment.
 */
public class AutoAlignConfig {

    /** Desired pixel X position of the target in the camera frame. */
    public final double targetX;

    /** Desired pixel Y position of the target in the camera frame. */
    public final double targetY;

    /** Minimum detected tag width (pixels) required before alignment begins. */
    public final double minWidth;

    /** Proportional gain for strafe (left/right) correction. */
    public final double kP_strafe;

    /** Proportional gain for forward/backward correction. */
    public final double kP_forward;

    /** Proportional gain for heading correction. */
    public final double kP_heading;

    /** Pixel tolerance for X axis (strafe). */
    public final double toleranceX;

    /** Pixel tolerance for Y axis (forward). */
    public final double toleranceY;

    /**
     * Creates a new AutoAlignConfig with all parameters specified.
     *
     * @param targetX     desired X pixel position of the target
     * @param targetY     desired Y pixel position of the target
     * @param minWidth    minimum tag width (pixels) to start alignment
     * @param kP_strafe   proportional gain for strafe correction
     * @param kP_forward  proportional gain for forward correction
     * @param kP_heading  proportional gain for heading correction
     * @param toleranceX  acceptable X-axis error in pixels
     * @param toleranceY  acceptable Y-axis error in pixels
     */
    public AutoAlignConfig(double targetX, double targetY, double minWidth,
                           double kP_strafe, double kP_forward, double kP_heading,
                           double toleranceX, double toleranceY) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.minWidth = minWidth;
        this.kP_strafe = kP_strafe;
        this.kP_forward = kP_forward;
        this.kP_heading = kP_heading;
        this.toleranceX = toleranceX;
        this.toleranceY = toleranceY;
    }
}
