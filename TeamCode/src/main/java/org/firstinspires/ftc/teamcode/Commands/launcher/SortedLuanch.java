package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class SortedLuanch extends SequentialCommandGroup {

    public SortedLuanch (LauncherBall launcher, Sorter sorter, HoldControl holdControl, long recoveryMs) {
        int startSlot = sorter.findStartIndex(
                RobotState.getInstance().getPattern(),
                RobotState.getInstance().getBallColors() );

        addCommands (
                new InstantCommand(() -> new TurnOneSlot(sorter, sorter.getTurnOffset(startSlot))),
                new ShootAllLoaded(launcher, sorter, holdControl, recoveryMs)
        );
    }

}
