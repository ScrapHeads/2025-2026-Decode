package org.firstinspires.ftc.teamcode.teleops.testing;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.drivetrain.DriveContinous;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

public class DriveOnly extends CommandOpMode {
    // Create all subsystems references
    GamepadEx driver = null;

    Drivetrain drivetrain = null;

    @Override
    public void initialize() {
        // Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        // Initialize the first controller
        // Can only have one active DRIVETRAIN controller at once
        driver = new GamepadEx(gamepad1);

        TimeTracker.setOffset();

        // Initialize the subsystems declared at the top of the code
        drivetrain = new Drivetrain(hm, new Pose2d());
        drivetrain.register();

        // Calling assignControls to set input commands
        assignControls();
    }

    public void assignControls() {
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

//        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> {
//            TelemetryPacket p = new TelemetryPacket();
//            p.put("Current Time:", TimeTracker.getTime());
//            dashboard.sendTelemetryPacket(p);
//        }));
    }

}
