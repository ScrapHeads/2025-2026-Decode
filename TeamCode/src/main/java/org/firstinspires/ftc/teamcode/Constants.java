package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.BallColor.GREEN;
import static org.firstinspires.ftc.teamcode.util.BallColor.PURPLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.vision.AutoAlignConfig;

import java.util.HashMap;
import java.util.Map;

@Config
public class Constants {
    public static HardwareMap hm;
    public static Telemetry tele;
    public static FtcDashboard dashboard;

    public static double intakePowerOffset = .25;

//
//    public static int[] PATTERN_TAG_IDS = {3, 2, 1};
//    public static Map<Integer, BallColor[]> patters = Map.of(
//            PATTERN_TAG_IDS[0], new BallColor[] {PURPLE, PURPLE, GREEN},
//            PATTERN_TAG_IDS[1], new BallColor[] {PURPLE, GREEN, PURPLE},
//            PATTERN_TAG_IDS[2], new BallColor[] {GREEN, PURPLE, PURPLE}
//    );

    //TODO find the pose of the tags on the field
    public static Pose2d redTagPose = new Pose2d(0,0, new Rotation2d(0));
    public static Pose2d blueTagPose = new Pose2d(0,0, new Rotation2d(0));

    // All in cm from front of robot for right now
    // Hood angle 1430
    public static double[][] data = {
            {70.0, 3200.0},
            {80.0, 3250.0},
            {90.0, 3275.0},
            {100.0, 3300.0},
            {110.0, 3325.0},
            {120.0, 3350.0},
            {130.0, 3400.0},
            {140.0, 3425.0},
            {150.0, 3450.0},
            {160.0, 3475.0},
            {170.0, 3500.0},
            {180.0, 0.0},
            {190.0, 0.0},
            {200.0, 3600.0},
            {210.0, 0.0},
            {220.0, 0.0},
            {230.0, 0.0},
            {240.0, 0.0},
            {250.0, 0.0},
            {260.0, 0.0},
            {270.0, 0.0},
            {280.0, 0.0},
            {290.0, 0.0},
            {300.0, 0.0},
            {310.0, 0.0},
            {320.0, 0.0},
            {330.0, 0.0},
            {340.0, 0.0},
            {350.0, 0.0},
            {360.0, 0.0}
    };

}
