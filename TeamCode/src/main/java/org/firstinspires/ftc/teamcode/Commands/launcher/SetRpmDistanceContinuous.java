package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class SetRpmDistanceContinuous extends CommandBase {

    private final Launcher launcher;

    public SetRpmDistanceContinuous (Launcher launcher) {
        this.launcher = launcher;

        addRequirements(launcher);
    }

    @Override
    public void initialize () {

    }

    @Override
    public void execute() {
        launcher.getAndSetFlywheelByDistance();
    }

    @Override
    public void end (boolean interrupted) {
        launcher.disable();
    }
}
