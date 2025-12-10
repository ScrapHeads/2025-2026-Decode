package org.firstinspires.ftc.teamcode.teleops.testing;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "ChochoTest", group = "ScrapHeads")
public class ChochoTest extends CommandOpMode {

    private GamepadEx driver;

    // continuous servos
    private Servo servo1;
    private Servo servo2;

    @Override
    public void initialize() {

        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        driver = new GamepadEx(gamepad1);

        servo1 = hm.get(Servo.class, "servo1");
        servo2 = hm.get(Servo.class, "servo2");

        assignControls();
    }

    public void assignControls() {

        // servo1 forward
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    servo1.setPosition(1.0);
                }));

        // servo1 reverse
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    servo1.setPosition(-1.0);
                }));

        // servo2 forward
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    servo2.setPosition(1.0);
                }));

        // servo2 reverse
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    servo2.setPosition(-1.0);
                }));

        // Stop both servos
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    servo1.setPosition(0);
                    servo2.setPosition(0);
                }));
    }
}
