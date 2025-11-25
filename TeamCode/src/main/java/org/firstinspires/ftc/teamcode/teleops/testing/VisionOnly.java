package org.firstinspires.ftc.teamcode.teleops.testing;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.StateIO;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

//@Disabled
@TeleOp(name = "VisionOnly", group = "ScrapHeads")
public class VisionOnly extends CommandOpMode {
    private GamepadEx driver;
    private Vision vision;
    private Drivetrain drivetrain;

    private final Pose2d startPose = new Pose2d(0,0, new Rotation2d());

    @Override
    public void initialize() {
        // Init constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        RobotState.getInstance().setAll(startPose, false, new BallColor[3], new ChassisSpeeds());

        driver = new GamepadEx(gamepad1);

        // Create drivetrain + vision subsystem
        drivetrain = new Drivetrain(hm, startPose);
        drivetrain.register();

        vision = new Vision(hm);
        vision.register();

        TimeTracker.setOffset();

        assignControls();
    }

    public void assignControls() {
        // Run passive vision detection (pose correction ON)
//        driver.getGamepadButton(A)
//                .whenPressed(new VisionDetection(vision, processor, drivetrain, true));

    }
}
