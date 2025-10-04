package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(Drivetrain.class)) {
            Drivetrain drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, new Rotation2d(0)));
            
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, new Rotation2d(0)))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        } else {
            throw new RuntimeException();
        }
    }
}
