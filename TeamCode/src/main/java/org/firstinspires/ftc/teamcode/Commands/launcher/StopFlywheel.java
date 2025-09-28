package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;

/**
 * One-shot command that disables the launcher PID and stops the motor.
 */
public class StopFlywheel extends CommandBase {
    private final LauncherBall launcher;

    public StopFlywheel(final LauncherBall launcher) {
        this.launcher = launcher;
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        launcher.disable();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
