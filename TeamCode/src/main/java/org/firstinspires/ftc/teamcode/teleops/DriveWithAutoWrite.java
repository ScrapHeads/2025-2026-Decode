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
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.StateIO;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "DriveWithAutoWrite", group = "ScrapHeads")
public class DriveWithAutoWrite extends CommandOpMode {
    // Create all subsystems references
    GamepadEx driver = null;

    Drivetrain drivetrain = null;

    public static boolean isBlue;

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

        // Get the state jason file
        RobotState rs = StateIO.load();

        if (rs != null) {
            // set the new drivetrain pos
            drivetrain.localizer.setPose(rs.pose);

            isBlue = rs.isBlue;

            // Use toString() to show what was loaded on Driver Station
            telemetry.addData("Loaded State", rs.toString());
            telemetry.update();

            // Clear so old data isn’t reused next match
            StateIO.clear();
        } else {
            telemetry.addLine("No saved state found, using default pose.");
            telemetry.update();
        }


        // Calling assignControls to set input commands
        assignControls();
    }

    public void assignControls() {
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));
    }

}
