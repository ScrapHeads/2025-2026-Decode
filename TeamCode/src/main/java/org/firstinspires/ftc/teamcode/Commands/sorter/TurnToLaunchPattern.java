package org.firstinspires.ftc.teamcode.Commands.sorter;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class TurnToLaunchPattern extends SequentialCommandGroup {
    private int startOffset;

    public TurnToLaunchPattern (Sorter sorter) {
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
                new TurnOneSlot(sorter, () -> startOffset)
        );
    }

}
