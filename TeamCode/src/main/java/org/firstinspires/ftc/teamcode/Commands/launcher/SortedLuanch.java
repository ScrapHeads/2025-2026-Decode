package org.firstinspires.ftc.teamcode.Commands.launcher;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class SortedLuanch extends SequentialCommandGroup {
    int startOffset;
    public SortedLuanch (LauncherBall launcher, Sorter sorter, HoldControl holdControl) {

        addCommands (
                new InstantCommand(() -> {
                    int dir = sorter.findStartOffset(
                            RobotState.getInstance().getPattern(),
                            RobotState.getInstance().getBallColors()
                    );
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Start direction", dir);
                    dashboard.sendTelemetryPacket(packet);

                    // Immediately schedule the correct one-step turn
                    new TurnOneSlot(sorter, dir).schedule();
                }),
//                new WaitUntilCommand(sorter::isAtSetPoint),
                new ShootAllLoaded(launcher, sorter, holdControl)
        );
    }

}
