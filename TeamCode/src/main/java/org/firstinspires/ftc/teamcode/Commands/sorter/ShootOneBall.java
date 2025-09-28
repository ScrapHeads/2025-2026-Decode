package org.firstinspires.ftc.teamcode.Commands.sorter;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class ShootOneBall extends CommandBase {

    private final Sorter sorter;
    private final LauncherBall launcher;

    public ShootOneBall (Sorter sorter, LauncherBall launcher) {
        this.sorter = sorter;
        this.launcher = launcher;

        addRequirements(sorter, launcher);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
