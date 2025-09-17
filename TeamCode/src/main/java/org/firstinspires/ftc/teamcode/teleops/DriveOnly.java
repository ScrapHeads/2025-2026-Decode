package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "DriveOnly", group = "ScrapHeads")
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

        // Initialize the subsystems declared at the top of the code
        drivetrain = new Drivetrain(hm, new Pose2d(0,0,0));
        drivetrain.register();

        // Calling assignControls to set input commands
        assignControls();
    }

    public void assignControls() {
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));
    }

}
