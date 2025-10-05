package org.firstinspires.ftc.teamcode.teleops.Examples;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.StateIO;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.BallColor;

@TeleOp(name = "DriveWithAutoWrite", group = "ScrapHeads")
public class DriveWithAutoWrite extends CommandOpMode {
    // Create all subsystems references
    GamepadEx driver = null;

    Drivetrain drivetrain = null;

//    RobotState robotState = null;

    @Override
    public void initialize() {
        // Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        // Initialize the first controller
        // Can only have one active DRIVETRAIN controller at once
        driver = new GamepadEx(gamepad1);

//        robotState = StateIO.load();
//
//        if (robotState == null) {
//            BallColor[] ballColors = new BallColor[] {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
//            robotState = new RobotState(new Pose2d(0,0, new Rotation2d(0)), false, ballColors);
//        }

        // Initialize the subsystems declared at the top of the code
        drivetrain = new Drivetrain(hm, new Pose2d(0,0, new Rotation2d(0)));
        drivetrain.register();

        // Calling assignControls to set input commands
        assignControls();
    }

    public void assignControls() {
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));
    }
}
