package org.firstinspires.ftc.teamcode.state;


import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.BallColor;

/**
 * Serializable snapshot of robot state to transfer from Auto to TeleOp.
 * Keep units consistent with your configuration
 * Project is set up as follows:
 * - Pose2d: inches and radians.
 * - Boolean for what alliance you are on sense there is only two
 */
public class RobotState {
    private int version = 1;

    // Pose on the field
    private Pose2d odometryPose;
    private Pose2d estimatedPose;

    private ChassisSpeeds chassisSpeeds;

    // get all the ball colors
    private BallColor[] ballColors;

    // If true on blue alliance if false on red alliance
    private boolean isBlue;

    private static RobotState instance;

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

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
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
    public RobotState(Pose2d pose, boolean isBlue, BallColor[] ballColors, ChassisSpeeds chassisSpeeds) {
        this.odometryPose = pose;
        this.estimatedPose = pose;
        this.isBlue = isBlue;
        this.chassisSpeeds = chassisSpeeds;

        this.ballColors = ballColors;
    }

    public Pose2d getOdometryPose() {return odometryPose;}

    public Pose2d getEstimatedPose() {return estimatedPose;}

    public void setOdometryPose(Pose2d newPose) {
        odometryPose = newPose;
        estimatedPose = newPose;
    }

    public void setChassisSpeeds (ChassisSpeeds chassisSpeeds) {this.chassisSpeeds = chassisSpeeds;}

    public ChassisSpeeds getChassisSpeeds() {return chassisSpeeds;}

    public BallColor[] getBallColors() {return ballColors;}

    public void setBallColorAtIndex(int index, BallColor ballColor) {
        ballColors[index] = ballColor;
    }

    public void setBallColors (BallColor[] ballColors) {this.ballColors = ballColors;}



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
                "pose=" + odometryPose +
                "estPose=" +estimatedPose+
                "isBlue=" + isBlue +
                "}";
    }
}
