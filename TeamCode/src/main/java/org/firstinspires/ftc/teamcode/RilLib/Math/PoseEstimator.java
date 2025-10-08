// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.RilLib.Math;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.TimeInterpolatableBuffer;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N1;
import org.firstinspires.ftc.teamcode.RilLib.Math.Numbers.N3;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

public class PoseEstimator {
    private final Matrix<N3, N1> m_q = new Matrix<>(N3.instance, N1.instance);
    private final Matrix<N3, N3> m_visionK = new Matrix<>(N3.instance, N3.instance);

    private static final double kBufferDuration = 1.5;
    // Maps timestamps to odometry-only pose estimates
    private final TimeInterpolatableBuffer<Pose2d> m_odometryPoseBuffer =
            TimeInterpolatableBuffer.createBuffer(kBufferDuration);
    // Maps timestamps to vision updates
    // Always contains one entry before the oldest entry in m_odometryPoseBuffer, unless there have
    // been no vision measurements after the last reset
    private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();

    private Pose2d m_odometryEstimate;
    private Pose2d m_poseEstimate;

    /**
     * Constructs a PoseEstimator.
     *
     * @param startingPose the pose the robot is at the time of construction
     * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
     *     in meters, and heading in radians). Increase these numbers to trust your state estimate
     *     less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *     the vision pose measurement less.
     */
    @SuppressWarnings("PMD.UnusedFormalParameter")
    public PoseEstimator(
            Pose2d startingPose,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        m_poseEstimate = startingPose;
        m_odometryEstimate = startingPose;

        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    @SuppressWarnings("PMD.UnusedFormalParameter")
    public PoseEstimator(Pose2d startingPose) {
        this(startingPose, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9));
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     *     theta]ᵀ, with units in meters and radians.
     */
    public final void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                        row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
            }
        }
    }

    /**
     * Resets the robot's pose.
     *
     * @param pose The pose to reset to.
     */
    public void resetPose(Pose2d pose) {
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = pose;
        m_odometryEstimate = pose;
    }

    /**
     * Resets the robot's translation.
     *
     * @param translation The pose to translation to.
     */
    public void resetTranslation(Translation2d translation) {
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = new Pose2d(translation, m_poseEstimate.getRotation());
        m_odometryEstimate = new Pose2d(translation, m_odometryEstimate.getRotation());
    }

    /**
     * Resets the robot's rotation.
     *
     * @param rotation The rotation to reset to.
     */
    public void resetRotation(Rotation2d rotation) {
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = new Pose2d(m_poseEstimate.getTranslation(), rotation);
        m_odometryEstimate = new Pose2d(m_odometryEstimate.getTranslation(), rotation);
    }

    /**
     * Gets the estimated robot pose.
     *
     * @return The estimated robot pose in meters.
     */
    public Pose2d getEstimatedPosition() {
        return m_poseEstimate;
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        // Step 0: If there are no odometry updates to sample, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        // Step 1: Make sure timestamp matches the sample from the odometry pose buffer. (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().lastKey();
        timestampSeconds =
                MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        // Step 2: If there are no applicable vision updates, use the odometry-only information.
        if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
            return m_odometryPoseBuffer.getSample(timestampSeconds);
        }

        // Step 3: Get the latest vision update from before or at the timestamp to sample at.
        double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
        VisionUpdate visionUpdate = m_visionUpdates.get(floorTimestamp);

        // Step 4: Get the pose measured by odometry at the time of the sample.
        Optional<Pose2d> odometryEstimate = m_odometryPoseBuffer.getSample(timestampSeconds);

        // Step 5: Apply the vision compensation to the odometry pose.
        return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
    }

    /** Removes stale vision updates that won't affect sampling. */
    private void cleanUpVisionUpdates() {
        // Step 0: If there are no odometry samples, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return;
        }

        // Step 1: Find the oldest timestamp that needs a vision update.
        double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();

        // Step 2: If there are no vision updates before that timestamp, skip.
        if (m_visionUpdates.isEmpty() || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
            return;
        }

        // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
        double newestNeededVisionUpdateTimestamp = m_visionUpdates.floorKey(oldestOdometryTimestamp);

        // Step 4: Remove all entries strictly before the newest timestamp we need.
        m_visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * PoseEstimator#updateWithTime} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()
                || m_odometryPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration
                > timestampSeconds) {
            return;
        }

        // Step 1: Clean up any old entries
        cleanUpVisionUpdates();

        // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
        Optional<Pose2d> odometrySample = m_odometryPoseBuffer.getSample(timestampSeconds);

        if (!odometrySample.isPresent()) {
            return;
        }

        // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
        // made.
        Optional<Pose2d> visionSample = sampleAt(timestampSeconds);

        if (!visionSample.isPresent()) {
            return;
        }

        // Step 4: Measure the twist between the old pose estimate and the vision pose.
        Twist2d twist = visionSample.get().log(visionRobotPoseMeters);

        // Step 5: We should not trust the twist entirely, so instead we scale this twist by a Kalman
        // gain matrix representing how much we trust vision measurements compared to our current pose.
        Matrix<N3, N1> k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

        // Step 6: Convert back to Twist2d.
        Twist2d scaledTwist =
                new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

        // Step 7: Calculate and record the vision update.
        VisionUpdate visionUpdate = new VisionUpdate(visionSample.get().exp(scaledTwist), odometrySample.get());
        m_visionUpdates.put(timestampSeconds, visionUpdate);

        // Step 8: Remove later vision measurements. (Matches previous behavior)
        m_visionUpdates.tailMap(timestampSeconds, false).entrySet().clear();

        // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
        // it's guaranteed to be the latest vision update.
        m_poseEstimate = visionUpdate.compensate(m_odometryEstimate);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method should be updated every loop
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will continue
     * to apply to future measurements until a subsequent call to {@link
     * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *     the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param deltaPose The twist from the last time you have updated the pose estimator (should be odometry only)
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d updateWithTime(double currentTimeSeconds, Twist2d deltaPose) {
        Pose2d odometryEstimate = m_odometryEstimate.exp(deltaPose);

        m_odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate);
        m_odometryEstimate = odometryEstimate;

        if (m_visionUpdates.isEmpty()) {
            m_poseEstimate = odometryEstimate;
        } else {
            VisionUpdate visionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
            m_poseEstimate = visionUpdate.compensate(odometryEstimate);
        }

        return getEstimatedPosition();
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param pose The new pose that your odometry believes you are in (should be odometry only)
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d updateWithTime(double currentTimeSeconds, Pose2d pose) {
        Pose2d odometryEstimate = pose;

        m_odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate);
        m_odometryEstimate = odometryEstimate;

        if (m_visionUpdates.isEmpty()) {
            m_poseEstimate = odometryEstimate;
        } else {
            VisionUpdate visionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
            m_poseEstimate = visionUpdate.compensate(odometryEstimate);
        }

        return getEstimatedPosition();
    }

    /**
     * Represents a vision update record. The record contains the vision-compensated pose estimate as
     * well as the corresponding odometry pose estimate.
     */
    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final Pose2d visionPose;

        // The pose estimated based solely on odometry.
        private final Pose2d odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
         * relative to this record's odometry pose to being relative to this record's vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public Pose2d compensate(Pose2d pose) {
            Transform2d delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }
}