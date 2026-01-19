package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class LauncherSetDefaultCommand extends CommandBase {

    private final Launcher launcher;

    public LauncherSetDefaultCommand(final Launcher launcher) {
        this.launcher = launcher;
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        launcher.setDefaultCommand(new SetRpmDistanceContinuous(launcher));
    }

    /**
     * This command runs once.
     * @return true once it is done with initialize
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Do not disable the launcher here; lifecycle is controlled externally.
     * This lets you chain shots or keep the wheel hot between shots.
     */
    @Override
    public void end(boolean interrupted) {
        // intentionally no-op
    }
}
