package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class AsistedLaunch extends SequentialCommandGroup {
    private int startOffset;
    public AsistedLaunch (Launcher launcher, Sorter sorter, HoldControl holdControl, Drivetrain drivetrain) {

        addCommands (
                //TODO Implement the auto turning
//                new TurnToTarget(drivetrain),
                new InstantCommand(launcher::getAndSetFlywheelByDistance),
                new SortedLuanch(launcher, sorter, holdControl)
        );
    }

}
