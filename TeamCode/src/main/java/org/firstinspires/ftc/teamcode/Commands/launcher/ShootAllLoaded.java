package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.HoldControlCommand;
import org.firstinspires.ftc.teamcode.Commands.sorter.TurnOneSlot;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class ShootAllLoaded extends SequentialCommandGroup {

    public ShootAllLoaded (LauncherBall launcher, Sorter sorter, HoldControl holdControl, long recoveryMs) {
        addCommands(
                new HoldControlCommand(holdControl, HoldControl.HoldPosition.LAUNCHING),
                new RepeatCommand(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(launcher::isReadyToLaunch),
                                        new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                                        new WaitUntilCommand(sorter::isAtSetPoint),
                                        new WaitCommand(recoveryMs)
                                ),
                                new TurnOneSlot(sorter, Sorter.CCW_DIRECTION),
                                () -> sorter.getCurrentColor().isBall()        // evaluated at runtime
                        )
                ).interruptOn(() -> RobotState.getInstance().isEmpty())
        );
    }

}
