package org.firstinspires.ftc.teamcode.commands.feeder;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FeederRail;

/**
 * Command to control the FeederRail subsystem by specifying a mode.
 *
 * <p>This command executes a single action: deploy, retract, or toggle.
 * It completes immediately after the action is issued.
 */
public class FeederRailCommand extends CommandBase {

    public enum Mode {
        DEPLOY,
        RETRACT,
        TOGGLE
    }

    private final FeederRail feederRail;
    private final Mode mode;

    /**
     * Constructs a FeederRailCommand.
     *
     * @param feederRail the FeederRail subsystem
     * @param mode       the desired mode (DEPLOY, RETRACT, TOGGLE)
     */
    public FeederRailCommand(FeederRail feederRail, Mode mode) {
        this.feederRail = feederRail;
        this.mode = mode;
        addRequirements(feederRail);
    }

    @Override
    public void initialize() {
        switch (mode) {
            case DEPLOY:
                feederRail.deploy();
                break;
            case RETRACT:
                feederRail.retract();
                break;
            case TOGGLE:
                feederRail.toggle();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // This is an instant command — finishes right after action
        return true;
    }
}
