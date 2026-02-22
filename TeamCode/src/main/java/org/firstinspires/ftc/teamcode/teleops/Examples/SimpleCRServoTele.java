package org.firstinspires.ftc.teamcode.teleops.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name = "SimpleCRServoTele", group = "Test")
public class SimpleCRServoTele extends LinearOpMode {

    private CRServo servo;

    @Override
    public void runOpMode() {

        // Map hardware
        servo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {

            // A = forward
            if (gamepad1.a) {
                servo.setPower(1.0);
            }
            // B = reverse
            else if (gamepad1.b) {
                servo.setPower(-1.0);
            }
            // Otherwise stop
            else {
                servo.setPower(0.0);
            }

            telemetry.addData("Servo Power", servo.getPower());
            telemetry.update();
        }
    }
}
