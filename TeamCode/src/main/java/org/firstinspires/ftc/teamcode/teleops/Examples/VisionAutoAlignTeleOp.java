package org.firstinspires.ftc.teamcode.teleops.Examples;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.VisionAutoAlign;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "AutoAlignTeleOp", group = "ScrapHeads")
public class VisionAutoAlignTeleOp extends CommandOpMode {
    private GamepadEx driver;
    private Vision vision;
    private Drivetrain drivetrain;

    private VisionAutoAlign autoAlignCommand;

    private final Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void initialize() {
        // Init constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        driver = new GamepadEx(gamepad1);

        // Init subsystems
        drivetrain = new Drivetrain(hm, startPose); // define in Constants if needed
        vision = new Vision(drivetrain);
        vision.register();

        assignControls();
    }

    private void assignControls() {
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            Constants.AUTO_ALIGN_ENABLED = !Constants.AUTO_ALIGN_ENABLED;

            if (Constants.AUTO_ALIGN_ENABLED) {
                // Start auto-align
                autoAlignCommand = new VisionAutoAlign(vision, drivetrain, driver);
                schedule(autoAlignCommand);
            } else {
                // Stop auto-align
                if (autoAlignCommand != null) {
                    autoAlignCommand.cancel();
                }
            }
        });
    }

    @Override
    public void run() {
        super.run();

        // Always display current toggle state
        tele.addData("AutoAlign", Constants.AUTO_ALIGN_ENABLED ? "ON" : "OFF");
        tele.update();
    }
}
