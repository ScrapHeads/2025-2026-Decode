package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Command that spins the flywheel to a target RPM and keeps it there
 * using the subsystem's internal PID + feedforward loop.
 * <p>
 * This command does NOT end when the shooter is "ready." It only ends
 * when the subsystem is explicitly disabled via {@link Launcher#disable()}.
 * That lets you hold speed across multiple shots until you decide to stop.
 */
public class SetFlywheelRpm extends CommandBase {

    /** Shooter subsystem providing PID control and readiness logic. */
    private final Launcher launcher;

    /** Target RPM to hold while this command is active. */
    private final double rpm;

    /**
     * Create a command to spin up and hold the flywheel at a given RPM.
     *
     * @param launcher the launcher subsystem
     * @param rpm      target speed in RPM (e.g., 6000)
     */
    public SetFlywheelRpm(final Launcher launcher, final double rpm) {
        this.launcher = launcher;
        this.rpm = rpm;
        addRequirements(launcher);
    }

    /**
     * Sets the target RPM and enables the subsystem's PID loop.
     * The subsystem ramps up from current speed to reduce brownouts.
     */
    @Override
    public void initialize() {
        launcher.setTargetRpm(rpm);
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
