package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.CAMERA_FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_LATERAL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_VERTICAL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_YAW_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.MIN_TAG_PIXEL_SIZE;
import static org.firstinspires.ftc.teamcode.Constants.TRACKED_TAG_IDS;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.vision.CameraParams;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;

import java.util.concurrent.TimeUnit;

/**
 * Vision subsystem for HuskyLens integration.
 * <p>
 * This subsystem manages:
 * <ul>
 *   <li>Communication with the HuskyLens sensor.</li>
 *   <li>Conversion of detected blocks into corrected robot poses using {@link VisionProcessor}.</li>
 *   <li>Rate-limited periodic updates to prevent overloading the I2C bus.</li>
 *   <li>Exposing the latest corrected pose to other commands and subsystems.</li>
 * </ul>
 */
public class Vision implements Subsystem {

    private final HuskyLens huskyLens;
    private final VisionProcessor processor;
    private Pose2d latestPose = null;

    // Rate limiting for sensor reads
    private static final int READ_PERIOD = 1; // seconds
    private final Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

    private final Drivetrain drivetrain;

    /**
     * Creates a Vision subsystem with HuskyLens and camera calibration parameters.
     *
     * @param drivetrain drivetrain reference used for motion compensation in vision correction
     */
    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        huskyLens = hm.get(HuskyLens.class, "huskylens");

        processor = new VisionProcessor(new CameraParams(
                CAMERA_FORWARD_OFFSET,
                CAMERA_LATERAL_OFFSET,
                CAMERA_VERTICAL_OFFSET,
                CAMERA_YAW_OFFSET
        ));

        rateLimit.expire();

        // Report connection status
        TelemetryPacket packet = new TelemetryPacket();
        if (!huskyLens.knock()) {
            packet.put("Vision", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            packet.put("Vision", "Connected, waiting for start...");
        }
        dashboard.sendTelemetryPacket(packet);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /**
     * Called periodically by the scheduler.
     * <p>
     * Performs a rate-limited HuskyLens read, filters blocks by tracked tag IDs
     * and minimum pixel size, and computes corrected poses using the vision processor.
     * Updates {@code latestPose} if a valid correction is available.
     */
    @Override
    public void periodic() {
        if (!rateLimit.hasExpired()) return;
        rateLimit.reset();

        HuskyLens.Block[] blocks = huskyLens.blocks();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Tag Count", blocks.length);

        for (HuskyLens.Block block : blocks) {
            boolean trackThis = false;
            for (int id : TRACKED_TAG_IDS) {
                if (block.id == id) {
                    trackThis = true;
                    break;
                }
            }

            if (!trackThis) continue;
            if (Math.min(block.width, block.height) < MIN_TAG_PIXEL_SIZE) continue;

            Pose2d correctedPose = processor.getCorrectedRobotPoseFromTag(drivetrain, block);
            if (correctedPose != null) {
                latestPose = correctedPose;
                packet.put("Corrected Pose", correctedPose);
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Manually sets the latest pose estimate from an external source.
     *
     * @param pose the new pose to store
     */
    public void setLatestPose(Pose2d pose) {
        this.latestPose = pose;
    }

    /**
     * Returns the most recently corrected pose.
     *
     * @return last corrected pose, or null if none available
     */
    public Pose2d getLatestPose() {
        return latestPose;
    }

    /**
     * Returns raw blocks reported by HuskyLens without correction or filtering.
     *
     * @return array of HuskyLens blocks
     */
    public HuskyLens.Block[] detectObject() {
        return huskyLens.blocks();
    }
}
