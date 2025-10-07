package org.firstinspires.ftc.teamcode.teleops.testing;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.launcher.SetFlywheelRpm;
import org.firstinspires.ftc.teamcode.Commands.launcher.StopFlywheel;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;

/**
 * Minimal TeleOp to test the LauncherBall PID/FF flywheel control.
 *
 * Controls:
 *   A = spin up to 6000 RPM (holds until disabled)
 *   B = disable launcher (stop motor)
 */
@Disabled
@TeleOp(name = "LauncherOnly", group = "ScrapHeads")
public class LauncherOnly extends CommandOpMode {
    // Controller
    private GamepadEx driver;

    // Subsystem
    private LauncherBall launcher;

    @Override
    public void initialize() {
        // Initialize shared constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        // Gamepad
        driver = new GamepadEx(gamepad1);

        // Subsystem
        launcher = new LauncherBall(hm);
        launcher.register();

        // Bind controls
        assignControls();

        tele.addLine("LauncherOnly initialized. A=Spin 6000 | B=Stop");
        tele.update();
    }

    private void assignControls() {
        // A: spin up and hold 6000 RPM (command ends only when launcher.disable() is called)
        driver.getGamepadButton(A)
                .whenPressed(new SetFlywheelRpm(launcher, 5000));

        // B: disable PID and stop motor
        driver.getGamepadButton(B)
                .whenPressed(new StopFlywheel(launcher));
    }
}
