package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.CAMERA_FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_LATERAL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_VERTICAL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_YAW_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.MIN_TAG_PIXEL_SIZE;
import static org.firstinspires.ftc.teamcode.Constants.TRACKED_TAG_IDS;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
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

    private final Limelight3A limelight;

    /**
     * Creates a Vision subsystem with HuskyLens and camera calibration parameters.
     *
     */
    public Vision(HardwareMap hm) {
        limelight = hm.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void startLimelight () {limelight.start();}
    public void stopLimeLight () {limelight.stop();}


    public void setPipeline (int index) {
        limelight.pipelineSwitch(index);
    }

    public Pose2d getPose () {
        LLResult limeLightResult = limelight.getLatestResult();
        //TODO Added pinpoint heading?
        return new Pose2d(limeLightResult.getTx(), limeLightResult.getTy(), new Rotation2d(0));
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

    }
}
