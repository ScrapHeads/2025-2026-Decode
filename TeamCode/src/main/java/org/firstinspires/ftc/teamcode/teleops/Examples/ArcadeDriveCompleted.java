package org.firstinspires.ftc.teamcode.teleops.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Tells the robot that it's in teleop, and the name that will show up on the drivers hub
@TeleOp(name="ArcadeDriveCompleted", group="FTCLessons")
// Makes the program not show up on the drivers hub, comment this out if you want to use the code
@Disabled
public class ArcadeDriveCompleted extends OpMode {
    private static DcMotor leftFront;
    private static DcMotor rightFront;


    // performs once when you hit the init button on the drivers hub
    @Override
    public void init() {
        // Finds the hardware who's configured name matches the name in the string
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Displays on the drivers hub when it finishes initialization
        telemetry.addData("status", "Initialized");
    }

    // called constantly while the program is running
    @Override
    public void loop() {
        double forwardsLeft = -gamepad1.left_stick_y;
        double forwardsRight = -gamepad1.right_stick_y;

        double leftFrontPower = forwardsLeft;
        double rightFrontPower = forwardsRight;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

        telemetry.addData("Motors", "leftFrontWheel:" + leftFront.getPower());
        telemetry.addData("Motors", "rightFrontWheel:" + rightFront.getPower());
        telemetry.addData("Input", "left y:" + gamepad1.left_stick_y +
                ", right x:" + gamepad1.right_stick_x);
    }
}
