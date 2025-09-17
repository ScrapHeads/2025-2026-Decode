package org.firstinspires.ftc.teamcode.state;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Serializable snapshot of robot state to transfer from Auto to TeleOp.
 * Keep units consistent with your configuration
 * Project is set up as follows:
 * - Pose2d: inches and radians.
 * - Boolean for what alliance you are on sense there is only two
 */
public class RobotState {
    public int version = 1;

    // Pose on the field
    public Pose2d pose;

    // If true on blue alliance if false on red alliance
    public boolean isBlue;

    /**
     * Required no-argument constructor.
     * Gson (and other serialization libraries) use this constructor when
     * deserializing JSON back into a RobotState object. Without it, calls
     * like {@code GSON.fromJson(...)} would fail because Java will not
     * generate a default constructor once a parameterized constructor is
     * defined. Typically not called directly in user code.
     */
    public RobotState() {

    }

    /**
     * Creates a new RobotState with the given pose and alliance flag.
     * This is typically used at the end of an Autonomous routine to capture
     * the robot's final position and alliance information so it can be saved
     * for use in TeleOp.
     *
     * @param pose   the robot's estimated field position and heading
     * @param isBlue true if on the blue alliance, false if on the red alliance
     */
    public RobotState(Pose2d pose, boolean isBlue) {
        this.pose = pose;
        this.isBlue = isBlue;
    }

    /**
     * Returns a human-readable string representation of this RobotState.
     * This is mainly used for debugging, logging, or telemetry so you can
     * quickly see the current values stored in the state (pose, alliance, etc.).
     *
     * @return a string showing the version, pose, and alliance flag
     */
    @Override
    public String toString() {
        return "RobotState{" +
                "version=" + version +
                "pose=" +pose +
                "isBlue=" + isBlue +
                "}";
    }
}
