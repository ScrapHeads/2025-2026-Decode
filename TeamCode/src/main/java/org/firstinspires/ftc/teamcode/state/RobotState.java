package org.firstinspires.ftc.teamcode.state;


import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Matrix;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N1;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;
import org.firstinspires.ftc.teamcode.RilLib.Math.PoseEstimator;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

import java.util.Vector;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Serializable snapshot of robot state to transfer from Auto to TeleOp.
 * Keep units consistent with your configuration
 * Project is set up as follows:
 * - Pose2d: inches and radians.
 * - Boolean for what alliance you are on sense there is only two
 */
public class RobotState {
    private int version = 1;

    private PoseEstimator poseEstimator = null;

    private Lock poseLock = new ReentrantLock();

    // Pose on the field
    private Pose2d odometryPose;
    private Pose2d estimatedPose;

    private ChassisSpeeds chassisSpeeds;

    // get all the ball colors
    private BallColor[] ballColors;
    private BallColor[] pattern;

    // If true on blue alliance if false on red alliance
    private Boolean isBlue;

    private boolean magSensorState;

    private static RobotState instance;

    /**
     * Required no-argument constructor.
     * Gson (and other serialization libraries) use this constructor when
     * deserializing JSON back into a RobotState object. Without it, calls
     * like {@code GSON.fromJson(...)} would fail because Java will not
     * generate a default constructor once a parameterized constructor is
     * defined. Typically not called directly in user code.
     */
    private RobotState() {}

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
    private RobotState(Pose2d pose, Boolean isBlue, BallColor[] ballColors, ChassisSpeeds chassisSpeeds) {
        this.odometryPose = pose;
        this.estimatedPose = pose;
        this.isBlue = isBlue;
        this.chassisSpeeds = chassisSpeeds;

        this.ballColors = ballColors;
    }

    public void setAll(Pose2d pose, Boolean isBlue, BallColor[] ballColors, ChassisSpeeds chassisSpeeds) {
        this.odometryPose = pose;
        this.estimatedPose = pose;
        this.isBlue = isBlue;
        this.chassisSpeeds = chassisSpeeds;

        this.ballColors = ballColors;
    }

    /**
     * Updates this RobotState from another RobotState instance.
     * Copies all relevant fields safely into the singleton.
     */
    public void setAll(RobotState other) {
        if (other == null) return;

        addOdometryObservation(other.getOdometryPose(), TimeTracker.getTime());
        this.estimatedPose = other.getEstimatedPose();
        this.isBlue = other.getTeam();
        this.chassisSpeeds = other.getChassisSpeeds();
        this.ballColors = other.getBallColors();
        this.pattern = other.getPattern();
    }

    public void setMagSensorState (boolean magSensorState) {this.magSensorState = magSensorState;}

    public boolean getMagSensorState () {return magSensorState;}

    public void setPattern(BallColor[] pattern) { this.pattern = pattern; }
    public BallColor[] getPattern () { return pattern; }

    public Boolean getTeam() {return isBlue;}

    public void setTeam(Boolean isBlue) {this.isBlue = isBlue;}

    public Pose2d getOdometryPose() {return odometryPose;}

    public Pose2d getEstimatedPose() {return estimatedPose;}

    public void addOdometryObservation(Pose2d newPose, double time) {
        poseLock.lock();

        odometryPose = newPose;

        if (poseEstimator == null) poseEstimator = new PoseEstimator(newPose);
        poseEstimator.updateWithTime(time, newPose);
        estimatedPose = poseEstimator.getEstimatedPosition();

        poseLock.unlock();
    }

    public void addVisionObservation(Pose2d visionPose, double time, Matrix<N3, N1> visionStdDevs) {
        if (poseEstimator == null) return;
        poseLock.lock();

        poseEstimator.addVisionMeasurement(visionPose, time, visionStdDevs);

        poseLock.unlock();
    }

    public void setChassisSpeeds (ChassisSpeeds chassisSpeeds) {this.chassisSpeeds = chassisSpeeds;}

    public ChassisSpeeds getChassisSpeeds() {return chassisSpeeds;}

    public BallColor[] getBallColors() {return ballColors;}

    public boolean hasEmpty() {
        for (BallColor ball : ballColors) {
            if (ball == BallColor.EMPTY) {
                return false;
            }
        }
        return true;
    }

    public boolean isEmpty() {
        for (BallColor ball : ballColors) {
            if (ball != BallColor.EMPTY) {
                return false;
            }
        }
        return true;
    }

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
