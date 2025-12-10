package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.HoldControlCommand;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class ShootAllLoaded extends SequentialCommandGroup {

    public ShootAllLoaded (Launcher launcher, Sorter sorter, HoldControl holdControl) {

        addCommands(
                new HoldControlCommand(holdControl, HoldControl.HoldPosition.LAUNCHING),
                new WaitCommand(300),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new WaitCommand(400),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new WaitCommand(500),
                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                new WaitUntilCommand(sorter::isAtSetPoint)
        );
//        addCommands(
//            new HoldControlCommand(holdControl, HoldControl.HoldPosition.LAUNCHING),
//
//            new RepeatCommand(
//                        new SequentialCommandGroup(
//                                new WaitUntilCommand(launcher::isReadyToLaunch),
//                                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
//                                new WaitUntilCommand(sorter::isAtSetPoint),
//                                new WaitCommand(recoveryMs),
//                                new InstantCommand(() -> loopCount[0]++) // increment counter
//                        )
//            ).interruptOn(() -> loopCount[0] >= 3) // stop after 3 loops
//        );
    }

}
