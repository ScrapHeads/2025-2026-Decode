package org.firstinspires.ftc.teamcode.vision;

/**
 * Container for camera mounting parameters relative to the robot frame.
 * <p>
 * These values describe how the camera is positioned and oriented
 * on the robot, and are used by {@link VisionProcessor} to convert
 * vision data into robot-relative coordinates.
 */
public class CameraParams {

    /** Forward offset from the robot center to the camera (inches). */
    public final double forwardOffset;

    /** Lateral offset from the robot center to the camera (inches, right is positive). */
    public final double lateralOffset;

    /** Vertical offset from the floor to the camera (inches). */
    public final double verticalOffset;

    /** Yaw (heading) offset of the camera relative to the robot forward (radians). */
    public final double yawOffset;

    /**
     * Creates a new set of camera parameters.
     *
     * @param forwardOffset  forward distance from robot center (inches)
     * @param lateralOffset  lateral distance from robot center (inches, right is positive)
     * @param verticalOffset height from floor to camera lens (inches)
     * @param yawOffset      yaw angle offset from robot forward (radians)
     */
    public CameraParams(double forwardOffset, double lateralOffset, double verticalOffset, double yawOffset) {
        this.forwardOffset = forwardOffset;
        this.lateralOffset = lateralOffset;
        this.verticalOffset = verticalOffset;
        this.yawOffset = yawOffset;
    }
}
