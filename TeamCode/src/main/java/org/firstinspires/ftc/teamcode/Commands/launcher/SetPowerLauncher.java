package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;

public class SetPowerLauncher extends CommandBase {
    /** Shooter subsystem providing PID control and readiness logic. */
    private final LauncherBall launcher;

    /** Target RPM to hold while this command is active. */
    private final double power;

    /**
     * Create a command to spin up and hold the flywheel at a given RPM.
     *
     * @param launcher the launcher subsystem
     * @param power      target speed in RPM -1 to 1
     */
    public SetPowerLauncher(LauncherBall launcher, double power) {
        this.launcher = launcher;
        this.power = power;
        addRequirements(launcher);
    }

    /**
     * Sets the target RPM and enables the subsystem's PID loop.
     * The subsystem ramps up from current speed to reduce brownouts.
     */
    @Override
    public void initialize() {
        launcher.setPower(power);
        launcher.enable();
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
