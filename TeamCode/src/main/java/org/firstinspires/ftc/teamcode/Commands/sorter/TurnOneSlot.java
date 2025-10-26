package org.firstinspires.ftc.teamcode.Commands.sorter;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

/**
 * Command to rotate the sorter exactly one slot using the magnetic sensor
 * for precise rotation detection rather than a fixed timer.
 *
 * <p>The command starts the rotation at the specified power and monitors
 * the magnetic sensor. When a full slot rotation is detected (sensor triggered
 * once), the sorter stops automatically.
 */
public class TurnOneSlot extends CommandBase {
    private final Sorter sorter;
    private final int direction;

    /**
     * @param sorter the sorter subsystem
     * @param direction either 1 turn CW or -1 turn CCW 0 does nothing
     * */
    public TurnOneSlot(Sorter sorter, int direction) {
        this.sorter = sorter;
        this.direction = direction;

        addRequirements(sorter);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        sorter.turnOneSlotDirection(direction);
    }

    @Override
    public boolean isFinished() {
        return sorter.isAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) { }
}
