package org.firstinspires.ftc.teamcode.teleops.testing;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_LATERAL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_VERTICAL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_YAW_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.ENABLE_POSE_CORRECTION;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.VisionDetection;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.CameraParams;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;

@TeleOp(name = "VisionOnly", group = "ScrapHeads")
public class VisionOnly extends CommandOpMode {
    private GamepadEx driver;
    private Vision vision;
    private Drivetrain drivetrain;
    private VisionProcessor processor;

    private final Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void initialize() {
        // Init constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        driver = new GamepadEx(gamepad1);

        // Create drivetrain + vision subsystem
        drivetrain = new Drivetrain(hm, startPose);
        vision = new Vision(drivetrain);
        vision.register();

        // Create processor using Constants
        processor = new VisionProcessor(new CameraParams(
                CAMERA_FORWARD_OFFSET,
                CAMERA_LATERAL_OFFSET,
                CAMERA_VERTICAL_OFFSET,
                CAMERA_YAW_OFFSET
        ));

        assignControls();
    }

    public void assignControls() {
        // Run passive vision detection (pose correction ON)
        driver.getGamepadButton(A)
                .whenPressed(new VisionDetection(vision, processor, drivetrain, true));

        // Toggle pose correction ON/OFF globally
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> {
                    ENABLE_POSE_CORRECTION = !ENABLE_POSE_CORRECTION;
                    tele.addData("PoseCorrection", ENABLE_POSE_CORRECTION ? "ENABLED" : "DISABLED");
                    tele.update();
                });
    }
}
