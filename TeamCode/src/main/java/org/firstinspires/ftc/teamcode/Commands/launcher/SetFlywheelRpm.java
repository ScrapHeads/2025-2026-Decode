package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;

public class SetFlywheelRpm extends CommandBase {
    private final LauncherBall launcher;
    private final double rpm;
    private final boolean waitUntilReady;

    public SetFlywheelRpm(LauncherBall launcher, double rpm, boolean waitUntilReady) {
        this.launcher = launcher;
        this.rpm = rpm;
        this.waitUntilReady = waitUntilReady;
        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        launcher.setTargetRpm(rpm);
        launcher.enable();
    }

    @Override
    public boolean isFinished() {
        return waitUntilReady && launcher.isReadyToLaunch();
    }

    @Override
    public void end(boolean interrupted) {
        // keep spinning if waitUntilReady==false (typical for teleop hold)
        // if you want it to stop on end, call launcher.disable() here
    }
}
