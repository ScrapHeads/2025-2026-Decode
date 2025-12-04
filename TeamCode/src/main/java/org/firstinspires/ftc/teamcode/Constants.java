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

    public static double intakePowerOffset = 0.0;


    public static int[] PATTERN_TAG_IDS = {21, 22, 23};
    public static Map<Integer, BallColor[]> patters = Map.of(
            PATTERN_TAG_IDS[0], new BallColor[] {GREEN, PURPLE, PURPLE},
            PATTERN_TAG_IDS[1], new BallColor[] {PURPLE, GREEN, PURPLE},
            PATTERN_TAG_IDS[2], new BallColor[] {PURPLE, PURPLE, GREEN}
    );

    //TODO find the pose of the tags on the field
    public static Pose2d redTagPose = new Pose2d(-1.482,1.413, new Rotation2d(-54));
    public static Pose2d blueTagPose = new Pose2d(-1.482,-1.413, new Rotation2d(54));

    // All in cm from front of robot for right now
    // Hood angle 1430
    public static double[][] data = {
            {80.0, 3375},
            {90.0, 3400},
            {100.0, 3435},
            {110.0, 3460},
            {120.0, 3490},
            {130.0, 3510},
            {140.0, 3530},
            {150.0, 3550},
            {160.0, 3570},
            {170.0, 3600},
            {180.0, 3635},
            {190.0, 3675},
            {200.0, 3710},
            {210.0, 3730},
            {220.0, 3760},
            {230.0, 3790},
            {240.0, 3820},
            {250.0, 3870},
            {260.0, 3930},
            {270.0, 4000},
            {280.0, 4110},
            {290.0, 4240},
            {300.0, 4390},
            {310.0, 4510},
            {320.0, 4630},
            {330.0, 4750},
    };

}
