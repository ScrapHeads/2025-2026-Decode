package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.util.ConversionUtil.convertPose2D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(Drivetrain.class)) {
            Drivetrain drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new ChassisSpeeds(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ,
                        -gamepad1.right_stick_x
                ));

                Pose2d pose = RobotState.getInstance().getOdometryPose();
                telemetry.addData("x", pose.getX());
                telemetry.addData("y", pose.getY());
                telemetry.addData("heading (deg)", pose.getRotation().toString());
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), convertPose2D(pose));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
