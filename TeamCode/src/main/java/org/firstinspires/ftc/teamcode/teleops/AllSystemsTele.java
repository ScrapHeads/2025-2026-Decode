package org.firstinspires.ftc.teamcode.teleops;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.subsystems.HoldControl.HoldPosition.LAUNCHING;
import static org.firstinspires.ftc.teamcode.subsystems.HoldControl.HoldPosition.LOADING;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveContinous;
import org.firstinspires.ftc.teamcode.Commands.HoldControlCommand;
import org.firstinspires.ftc.teamcode.Commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.Commands.launcher.SetFlywheelRpm;
import org.firstinspires.ftc.teamcode.Commands.launcher.StopFlywheel;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;
import org.firstinspires.ftc.teamcode.subsystems.LauncherHood;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.util.BallColor;

@TeleOp(name = "AllSystemsTele", group = "ScrapHeads")
public class AllSystemsTele extends CommandOpMode {
    // Controller
    private GamepadEx driver;

    // Subsystem
    private Drivetrain drivetrain;
    private LauncherBall launcher;
    private Sorter sorter;
//    private FeederRail feederRail;
    private HoldControl holdControl;
    private Intake intake;
    private LauncherHood hood;

    @Override
    public void initialize() {
        // Initialize shared constants
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        // Initialize the subsystems declared at the top of the code
        drivetrain = new Drivetrain(hm, new Pose2d());
        drivetrain.register();

        // Gamepad
        driver = new GamepadEx(gamepad1);

        // Subsystem
        launcher = new LauncherBall(hm);
        launcher.register();

        sorter = new Sorter(hm, 0, new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN});
        sorter.register();

//        feederRail = new FeederRail(hm);
//        feederRail.register();

        holdControl = new HoldControl(hm);
        holdControl.register();

        intake = new Intake(hm);
        intake.register();

        hood = new LauncherHood(hm);
        hood.register();

        // Bind controls
        assignControls();

        tele.addLine("LauncherOnly initialized. A=Spin 5000 | B=Stop");
        tele.addData("What index for sorter: ", sorter.getCurrentIndex());
        tele.addData("What index for sorter: ", sorter.getCurrentColor());
        tele.addData("Is colored ball: ", sorter.getCurrentColor().isBall());
        tele.update();
    }

    private void assignControls() {
        // Set up continuous drive
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

        // A: spin up and hold 6000 RPM (command ends only when launcher.disable() is called)
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .1)
                .whenActive(new SetFlywheelRpm(launcher, 5000));

        // Left Trigger: disable PID and stop motor
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .1)
                .whenActive(new StopFlywheel(launcher));

        driver.getGamepadButton(A)
                .whenPressed(new RunIntakeCommand(intake, Intake.INTAKE_POWER));

        driver.getGamepadButton(B)
                .whenPressed(new RunIntakeCommand(intake, 0));

        driver.getGamepadButton(X)
                .whenPressed(new TurnOneSlot(sorter, -0.5));

        driver.getGamepadButton(Y)
                        .whenPressed(shootAllLoaded(launcher, sorter, 100));

        driver.getGamepadButton(DPAD_DOWN)
                .whenPressed(new HoldControlCommand(holdControl, LAUNCHING));

//        driver.getGamepadButton(DPAD_UP)
//                .whenPressed(new SetHoodAngleCommand(hood, 0));
    }

    public static Command shootAllLoaded(LauncherBall launcher, Sorter sorter, long recoveryMs) {
        TelemetryPacket packet = new TelemetryPacket();
        SequentialCommandGroup command = new SequentialCommandGroup();

        for (int offset = 0; offset < Sorter.SLOT_COUNT; offset++) {
            command.addCommands(
//                    new FeederRailCommand(FeederRail, FeederRailCommand.Mode.DEPLOY),

                    // if there is a ball in the current slot, shoot it
                    // if not turn to the next slot to run again
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(launcher::isReadyToLaunch),
                                    new TurnOneSlot(sorter, -1),
                                    new WaitCommand(recoveryMs),
                                    new WaitUntilCommand(launcher::isReadyToLaunch)
                            ),
                            new TurnOneSlot(sorter, Sorter.CCW_POWER),
                            () -> sorter.getCurrentColor().isBall()        // evaluated at runtime
                    )
            );
        }
        dashboard.sendTelemetryPacket(packet);
        return command;
    }
}

