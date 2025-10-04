package org.firstinspires.ftc.teamcode.teleops.testing;


import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.launcher.SetFlywheelRpm;
import org.firstinspires.ftc.teamcode.Commands.launcher.StopFlywheel;
import org.firstinspires.ftc.teamcode.Commands.sorter.ShootOneBall;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.util.BallColor;

@TeleOp(name = "LauncherSorterTele", group = "ScrapHeads")
public class LauncherSorterTele extends CommandOpMode {
    // Controller
    private GamepadEx driver;

    // Subsystem
    private LauncherBall launcher;
    private Sorter sorter;

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

        sorter = new Sorter(hm, 0, new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN});
        sorter.register();

        // Bind controls
        assignControls();

        tele.addLine("LauncherOnly initialized. A=Spin 5000 | B=Stop");
        tele.addData("What index for sorter: ", sorter.getCurrentIndex());
        tele.addData("What index for sorter: ", sorter.getCurrentColor());
        tele.addData("Is colored ball: ", sorter.getCurrentColor().isBall());
        tele.update();
    }

    private void assignControls() {
        // A: spin up and hold 6000 RPM (command ends only when launcher.disable() is called)
        driver.getGamepadButton(A)
                .whenPressed(new SetFlywheelRpm(launcher, 5000));

        // B: disable PID and stop motor
        driver.getGamepadButton(B)
                .whenPressed(new StopFlywheel(launcher));

        driver.getGamepadButton(X)
                .whenPressed(shootAllLoaded(launcher, sorter, Sorter.STEP_MS));
    }

    public static Command shootAllLoaded(LauncherBall launcher, Sorter sorter, long recoveryMs) {
        int start = sorter.getCurrentIndex();
        TelemetryPacket packet = new TelemetryPacket();
        SequentialCommandGroup burst = new SequentialCommandGroup();

        for (int offset = 0; offset < Sorter.SLOT_COUNT; offset++) {
            final int idx = (start + offset) % Sorter.SLOT_COUNT;
            dashboard.sendTelemetryPacket(packet);
            burst.addCommands(
                    // move at most one step to the next index in the scan
                    new TurnOneSlot(sorter, Sorter.CCW_POWER),
                    new InstantCommand(() -> packet.addLine("Made it")),
                    new InstantCommand(() -> dashboard.sendTelemetryPacket(packet)),

                    // if there is a ball in the current slot, shoot it
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(launcher::isReadyToLaunch),
                                    new TurnOneSlot(sorter, -1),
                                    new WaitCommand(recoveryMs),
                                    new WaitUntilCommand(launcher::isReadyToLaunch)
                            ),
                            new InstantCommand(),
                            () -> sorter.getCurrentColor().isBall()        // evaluated at runtime
                    )
            );
        }
        dashboard.sendTelemetryPacket(packet);
        return burst;
    }
}
