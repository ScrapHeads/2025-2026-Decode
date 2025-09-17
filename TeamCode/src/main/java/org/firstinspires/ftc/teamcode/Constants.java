package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.AutoAlignConfig;

import java.util.HashMap;
import java.util.Map;

@Config
public class Constants {
    public static HardwareMap hm;
    public static Telemetry tele;
    public static FtcDashboard dashboard;

    // --------------------------
    // Vision system toggles
    // --------------------------
    public static boolean ENABLE_POSE_CORRECTION = false; // if true, vision pose updates localizer
    // --------------------------
    // Camera setup (mounting)
    // --------------------------
    public static double CAMERA_FORWARD_OFFSET = 0.0;  // inches from robot center forward
    public static double CAMERA_LATERAL_OFFSET = 0.0;  // inches from robot center left (+)
    public static double CAMERA_VERTICAL_OFFSET = 0.0; // inches from floor
    public static double CAMERA_YAW_OFFSET = 0.0;      // radians yaw relative to robot

    // --------------------------
    // Camera properties
    // --------------------------
    public static final int IMAGE_WIDTH = 320;                        // HuskyLens QVGA default
    public static final double CAMERA_HORIZONTAL_FOV = Math.toRadians(60); // ~60° FOV

    // --------------------------
    // Vision calibration
    // --------------------------
    public static double DISTANCE_CONSTANT = 500.0; // forward distance scaling
    public static double LEFT_SCALING = 12.0;       // left/right scaling

    // --------------------------
    // Filtering
    // --------------------------
    public static double MIN_TAG_PIXEL_SIZE = 20.0; // ignore tiny detections
    public static int[] TRACKED_TAG_IDS = {1, 2, 3};

    // --------------------------
    // Auto-align system
    // --------------------------
    public static boolean AUTO_ALIGN_ENABLED = false; // dashboard + driver toggle

    // --------------------------
    // Auto-align configs per tag
    // --------------------------
    public static Map<Integer, AutoAlignConfig> TAG_CONFIGS =
            new HashMap<>();

    static {
        // Example tag 1: tight X, looser Y
        TAG_CONFIGS.put(1, new AutoAlignConfig(
                IMAGE_WIDTH / 2.0, 120, 40,   // targetX, targetY, minWidth
                0.01, 0.01, 0.005,           // kP_strafe, kP_forward, kP_heading
                8, 15                        // toleranceX, toleranceY
        ));

        // Example tag 2: looser X, tighter Y
        TAG_CONFIGS.put(2, new AutoAlignConfig(
                IMAGE_WIDTH / 2.0, 130, 50,
                0.012, 0.015, 0.006,
                12, 6
        ));
    }

}
