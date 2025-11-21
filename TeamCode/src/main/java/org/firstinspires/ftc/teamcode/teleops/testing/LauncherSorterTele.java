package org.firstinspires.ftc.teamcode.teleops.testing;


import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.launcher.SetFlywheelRpm;
import org.firstinspires.ftc.teamcode.Commands.launcher.StopFlywheel;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

import java.util.Arrays;

@Disabled
@TeleOp(name = "LauncherSorterTele", group = "ScrapHeads")
public class LauncherSorterTele extends CommandOpMode {
    // Controller
    private GamepadEx driver;

    // Subsystem
    private Launcher launcher;
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
        launcher = new Launcher(hm);
        launcher.register();

        sorter = new Sorter(hm);
        sorter.register();

        // Bind controls
        assignControls();

        tele.addData("What index for sorter: ", sorter.getCurrentIndex());
        tele.addData("Color for sorter: ", Arrays.toString(RobotState.getInstance().getBallColors()));
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
                .whenPressed(shootAllLoaded(launcher, sorter, 300));
    }

    public static Command shootAllLoaded(Launcher launcher, Sorter sorter, long recoveryMs) {
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
                            new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                            () -> sorter.getCurrentColor().isBall()        // evaluated at runtime
                    )
            );
        }
        dashboard.sendTelemetryPacket(packet);
        return command;
    }
}
