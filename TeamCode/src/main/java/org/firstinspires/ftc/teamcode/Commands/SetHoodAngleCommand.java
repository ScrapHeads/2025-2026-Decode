package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LauncherHood;

/**
 * Command to set the LauncherHood to a specific target angle.
 *
 * <p>This command is instant – it sends the servo to the desired
 * angle and completes immediately.
 */
public class SetHoodAngleCommand extends CommandBase {

    private final LauncherHood hood;
    private final double targetAngle;

    /**
     * Constructs a new SetHoodAngleCommand.
     *
     * @param hood The LauncherHood subsystem
     * @param targetAngle The angle (degrees) to turn the hood to
     */
    public SetHoodAngleCommand(LauncherHood hood, double targetAngle) {
        this.hood = hood;
        this.targetAngle = targetAngle;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }
}
