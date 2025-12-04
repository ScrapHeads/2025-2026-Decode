package org.firstinspires.ftc.teamcode.Commands.launcher;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.util.BallColor.EMPTY;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.HoldControlCommand;
import org.firstinspires.ftc.teamcode.Commands.sorter.ClearSorterSlots;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class SortedLuanchExtraSpin extends SequentialCommandGroup {
    private int startOffset;
    public SortedLuanchExtraSpin(Launcher launcher, Sorter sorter, HoldControl holdControl) {

        addCommands (
                new InstantCommand(() -> {
                    startOffset = sorter.findStartOffset(
                            RobotState.getInstance().getPattern(),
                            RobotState.getInstance().getBallColors()
                    );
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Start direction", startOffset);
                    dashboard.sendTelemetryPacket(packet);
                }),
                new TurnOneSlot(sorter, () -> startOffset),
                new WaitCommand(500),

                new HoldControlCommand(holdControl, HoldControl.HoldPosition.LAUNCHING),
                new WaitCommand(100),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new WaitUntilCommand(sorter::isAtSetPoint),
//                new WaitUntilCommand(launcher::isReadyToLaunch),
                new WaitCommand(300),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new WaitUntilCommand(sorter::isAtSetPoint),
//                new WaitUntilCommand(launcher::isReadyToLaunch),
                new WaitCommand(300),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new WaitUntilCommand(sorter::isAtSetPoint),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new WaitUntilCommand(sorter::isAtSetPoint),
                new ClearSorterSlots(sorter)
        );
    }

}
